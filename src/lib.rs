//! This is a platform-agnostic Rust driver for the TMP108 temperature sensor
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.ti.com/lit/gpn/tmp108
//!
//! # Operational notes
//!
//! ## I²C bus ownership
//!
//! The driver assumes single-master ownership of the TMP108. Several
//! methods (notably `configure`, `one_shot`, and `shutdown` on both
//! [`Tmp108`] and [`AsyncTmp108`]) perform a read-modify-write on the
//! configuration register as two distinct I²C transactions. On a
//! multi-master bus, a second master writing to register `0x01`
//! between the read and the write will silently lose those writes —
//! the TMP108 has no register-level lock. In multi-master designs,
//! serialise driver access at a higher level (e.g. a bus mutex around
//! the entire driver, not just individual I²C transactions).
//!
//! ## Driver lifecycle on drop
//!
//! Dropping a [`Tmp108`], [`AsyncTmp108`] or [`AlertTmp108`] does
//! **not** change the chip's operating mode. The chip retains whatever
//! `M` bits were last written. If you want the chip to stop drawing
//! current after the driver goes out of scope, call
//! [`Tmp108::shutdown`] or [`AsyncTmp108::shutdown`] (or finish an
//! [`AsyncTmp108::continuous`] call cleanly) before dropping.
//!
//! In particular, dropping the future returned by
//! [`AsyncTmp108::continuous`] mid-flight (e.g. via
//! `embassy_futures::select!` or `tokio::time::timeout`) leaves the
//! chip in `Mode::Continuous` indefinitely. See the cancel-safety note
//! on that method.

#![doc(html_root_url = "https://docs.rs/tmp108/latest")]
#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "async")]
use device_driver::AsyncRegisterInterface;
use device_driver::{FieldsetMetadata, RegisterInterface, RegisterInterfaceBase};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

#[allow(clippy::all)]
#[allow(clippy::pedantic)]
#[allow(missing_docs)]
#[allow(unsafe_code)]
#[allow(unused)]
mod inner;

pub use crate::inner::{ConversionRate, Hysteresis, Mode, Polarity, Thermostat};
use crate::inner::{Inner, THigh, TLow};
pub use crate::ops::{Celsius, OutOfRange};

/// A0 pin logic level representation.
#[derive(Debug, Default)]
pub enum A0 {
    /// A0 tied to GND (default).
    #[default]
    Gnd,
    /// A0 tied to V+.
    Vplus,
    /// A0 tied to SDA.
    Sda,
    /// A0 tied to SCL.
    Scl,
}

impl From<A0> for u8 {
    fn from(connection: A0) -> Self {
        match connection {
            A0::Gnd => 0b100_1000,
            A0::Vplus => 0b100_1001,
            A0::Sda => 0b100_1010,
            A0::Scl => 0b100_1011,
        }
    }
}

/// Tmp108 configuration parameters
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Config {
    /// Thermostat mode.
    pub thermostat_mode: Thermostat,
    /// Alert pin polarity.
    pub alert_polarity: Polarity,
    /// Conversion rate.
    pub conversion_rate: ConversionRate,
    /// Temperature hysteresis.
    pub hysteresis: Hysteresis,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            thermostat_mode: Thermostat::Comparator,
            alert_polarity: Polarity::ActiveLow,
            conversion_rate: ConversionRate::OneHz,
            hysteresis: Hysteresis::OneC,
        }
    }
}

/// Pure-function register codec.
///
/// All sync/async-agnostic chip logic lives here: the [`Celsius`]
/// temperature newtype and its register codec, snapping of
/// continuous-f32 hysteresis values to the four discrete chip
/// settings, and the conversion between the typed [`Config`] and the
/// generated `Configuration` field-set.
///
/// The blocking and async drivers both delegate to this module so the
/// meaningful work lives in exactly one place; the per-driver methods
/// are thin shells that perform I²C and call into `ops`.
///
/// This module is pure: no bus, no `async`, no HAL. Every function
/// here is total or explicitly fallible, and the domains are small
/// enough that the tests walk them exhaustively rather than sampling.
pub(crate) mod ops {
    use crate::Config;
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    use crate::Hysteresis;
    use crate::inner::Configuration;

    /// Documented power-on reset value of the configuration register.
    /// Used by [`crate::Tmp108::probe`] to verify chip presence.
    pub(crate) const POR_CONFIG: u16 = 0x1022;

    /// Tolerance band for snapping continuous-f32 hysteresis input
    /// to the four discrete chip settings, in °C.
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    pub(crate) const HYSTERESIS_TOLERANCE: f32 = 0.05;

    /// A temperature the TMP108 can represent.
    ///
    /// Stored as sixteenths of a degree Celsius, which is the part's
    /// native resolution: one LSB is 0.0625 °C. The representable
    /// range is the span of the sensor's 12-bit two's complement
    /// field, `-2048..=2047` sixteenths, or [`Celsius::MIN`] to
    /// [`Celsius::MAX`].
    ///
    /// That is 4096 inhabitants, every one of them a temperature the
    /// part can actually report or accept as a limit. An `f32` in the
    /// same position would admit roughly four billion, including
    /// `NaN`, both infinities, and −400 °C.
    ///
    /// # Examples
    ///
    /// ```
    /// use tmp108::Celsius;
    /// let t = Celsius::try_from_degrees(25.0).unwrap();
    /// assert_eq!(t.sixteenths(), 400);
    /// assert_eq!(t.to_degrees(), 25.0);
    /// ```
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Celsius(i16);

    /// Why a value could not be converted into a [`Celsius`].
    ///
    /// This is the only fallible direction in the module. Bytes
    /// arriving from the device are always a valid temperature, so the
    /// failures here all come from the human-facing side of the
    /// boundary.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    #[non_exhaustive]
    pub enum OutOfRange {
        /// The value was `NaN`.
        NotANumber,
        /// The value was below [`Celsius::MIN`].
        TooLow,
        /// The value was above [`Celsius::MAX`].
        TooHigh,
    }

    impl core::fmt::Display for Celsius {
        /// Renders as degrees Celsius, honouring precision: `{:.2}` works.
        ///
        /// This is the rendering edge the post-parse value is finally
        /// allowed to become a float at.
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            core::fmt::Display::fmt(&self.to_degrees(), f)
        }
    }

    impl core::fmt::Display for OutOfRange {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::NotANumber => f.write_str("temperature is not a number"),
                Self::TooLow => f.write_str("temperature is below -128 C"),
                Self::TooHigh => f.write_str("temperature is above 127.9375 C"),
            }
        }
    }

    /// Sixteenths of a degree in the lowest representable temperature.
    pub(crate) const MIN_SIXTEENTHS: i16 = -2048;

    /// Sixteenths of a degree in the highest representable temperature.
    pub(crate) const MAX_SIXTEENTHS: i16 = 2047;

    /// Bits the 12-bit temperature field is left-shifted by within the
    /// register.
    const REGISTER_SHIFT: u32 = 4;

    /// Values in sixteenths at or below this round to something below
    /// [`Celsius::MIN`].
    pub(crate) const LOWEST_ACCEPTED: f32 = -2048.5;

    /// Values in sixteenths at or above this round to something above
    /// [`Celsius::MAX`].
    pub(crate) const HIGHEST_ACCEPTED: f32 = 2047.5;

    impl Celsius {
        /// Lowest temperature the part can represent, −128 °C.
        pub const MIN: Self = Self(MIN_SIXTEENTHS);

        /// Highest temperature the part can represent, +127.9375 °C.
        ///
        /// Note this is one LSB short of +128 °C: two's complement
        /// affords one more negative code than positive.
        pub const MAX: Self = Self(MAX_SIXTEENTHS);

        /// Zero degrees.
        pub const ZERO: Self = Self(0);

        /// Build a temperature from sixteenths of a degree.
        ///
        /// # Errors
        ///
        /// [`OutOfRange`] if `sixteenths` is outside `-2048..=2047`.
        ///
        /// # Examples
        ///
        /// ```
        /// use tmp108::{Celsius, OutOfRange};
        /// assert_eq!(Celsius::from_sixteenths(400).unwrap().to_degrees(), 25.0);
        /// assert_eq!(Celsius::from_sixteenths(9999), Err(OutOfRange::TooHigh));
        /// ```
        pub const fn from_sixteenths(sixteenths: i16) -> Result<Self, OutOfRange> {
            if sixteenths < MIN_SIXTEENTHS {
                Err(OutOfRange::TooLow)
            } else if sixteenths > MAX_SIXTEENTHS {
                Err(OutOfRange::TooHigh)
            } else {
                Ok(Self(sixteenths))
            }
        }

        /// Parse a temperature in degrees Celsius.
        ///
        /// This is the boundary between what a human writes and what
        /// the part can store. Values are rounded to the nearest
        /// sixteenth of a degree, half away from zero.
        ///
        /// # Errors
        ///
        /// [`OutOfRange`] if `degrees` is `NaN`, infinite, or rounds to
        /// a value outside [`Celsius::MIN`]`..=`[`Celsius::MAX`].
        ///
        /// # Examples
        ///
        /// ```
        /// use tmp108::{Celsius, OutOfRange};
        /// assert_eq!(Celsius::try_from_degrees(-55.0).unwrap().sixteenths(), -880);
        /// assert_eq!(Celsius::try_from_degrees(128.0), Err(OutOfRange::TooHigh));
        /// assert_eq!(Celsius::try_from_degrees(f32::NAN), Err(OutOfRange::NotANumber));
        /// ```
        pub fn try_from_degrees(degrees: f32) -> Result<Self, OutOfRange> {
            if degrees.is_nan() {
                return Err(OutOfRange::NotANumber);
            }

            let scaled = degrees * 16.0;

            // Rounding half away from zero then truncating means a value is
            // representable exactly when it lies strictly inside this open
            // interval. Checking `scaled` rather than the rounded value keeps
            // the endpoints, -128.0 and 127.9375, inside the range where they
            // belong.
            if scaled <= LOWEST_ACCEPTED {
                return Err(OutOfRange::TooLow);
            }
            if scaled >= HIGHEST_ACCEPTED {
                return Err(OutOfRange::TooHigh);
            }

            // `f32::round` lives in `std`, and this crate is `no_std`. Adding
            // a half and truncating toward zero rounds half away from zero.
            let rounded = if scaled >= 0.0 { scaled + 0.5 } else { scaled - 0.5 };

            // The bounds above guarantee this lands in `-2048..=2047`.
            #[allow(clippy::cast_possible_truncation)]
            Ok(Self(rounded as i16))
        }

        /// The temperature in sixteenths of a degree.
        #[must_use]
        pub const fn sixteenths(self) -> i16 {
            self.0
        }

        /// The temperature in degrees Celsius.
        ///
        /// Exact: the scale factor is a power of two and the value
        /// always fits in an `f32` mantissa, so nothing is rounded
        /// here.
        #[must_use]
        pub fn to_degrees(self) -> f32 {
            f32::from(self.0) * 0.0625
        }

        /// Decode a temperature or limit register.
        ///
        /// Total. The register holds a 12-bit two's complement value
        /// left-justified in 16 bits, so every one of the 65,536
        /// possible bit patterns names a temperature in range. The four
        /// unused low bits are discarded — per the datasheet (Table 6
        /// and Table 12) they are hardwired zero and "always read 0".
        pub(crate) const fn from_register(raw: [u8; 2]) -> Self {
            // Arithmetic shift: sign-extends, and floors rather than
            // truncating toward zero, which matters only for the low bits the
            // part never sets.
            Self(i16::from_be_bytes(raw) >> REGISTER_SHIFT)
        }

        /// Encode into a temperature or limit register.
        ///
        /// Total. The invariant on the inner value guarantees the shift
        /// cannot overflow.
        pub(crate) const fn to_register(self) -> [u8; 2] {
            (self.0 << REGISTER_SHIFT).to_be_bytes()
        }
    }

    /// Snap a continuous-f32 hysteresis input to the nearest legal
    /// chip setting within the [`HYSTERESIS_TOLERANCE`] band.
    ///
    /// Returns `None` for inputs that are non-finite or further than
    /// the tolerance from every legal setting. The four legal
    /// settings are 0, 1, 2, and 4 °C.
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    pub(crate) fn snap_hysteresis(input: f32) -> Option<Hysteresis> {
        const HYS_VALUES: &[(f32, Hysteresis)] = &[
            (0.0, Hysteresis::ZeroC),
            (1.0, Hysteresis::OneC),
            (2.0, Hysteresis::TwoC),
            (4.0, Hysteresis::FourC),
        ];

        if !input.is_finite() {
            return None;
        }

        // HYS_VALUES is non-empty so min_by always returns Some.
        let (closest, snapped) = HYS_VALUES
            .iter()
            .copied()
            .min_by(|(a, _), (b, _)| (input - a).abs().total_cmp(&(input - b).abs()))
            .expect("HYS_VALUES is non-empty");

        if (input - closest).abs() > HYSTERESIS_TOLERANCE {
            return None;
        }

        Some(snapped)
    }

    /// Decode a configuration-register snapshot into a typed [`Config`].
    pub(crate) fn decode_config(c: Configuration) -> Config {
        Config {
            thermostat_mode: c.tm(),
            alert_polarity: c.pol(),
            conversion_rate: c.cr(),
            hysteresis: c.hys(),
        }
    }

    /// Apply a typed [`Config`] to a configuration-register snapshot,
    /// preserving untouched bits (M, FL, FH, ID).
    pub(crate) fn apply_config(r: &mut Configuration, cfg: Config) {
        r.set_tm(cfg.thermostat_mode);
        r.set_pol(cfg.alert_polarity);
        r.set_cr(cfg.conversion_rate);
        r.set_hys(cfg.hysteresis);
    }
}

/// Tmp108 device driver.
/// Blocking TMP108 driver.
///
/// Built on [`embedded_hal::i2c::I2c`]. The driver owns the I2C bus
/// for the lifetime of any operation; you can recover the bus by
/// calling [`destroy`][Self::destroy].
///
/// For the asynchronous flavor, see [`AsyncTmp108`].
pub struct Tmp108<I2C: I2c> {
    inner: Inner<Interface<I2C>>,
    addr: u8,
}

/// Asynchronous TMP108 driver.
///
/// Built on [`embedded_hal_async::i2c::I2c`]. The async flavor unlocks
/// [`AsyncTmp108::continuous`] (which has no blocking equivalent) and
/// is required by [`AlertTmp108`].
///
/// For the blocking flavor, see [`Tmp108`].
#[cfg(feature = "async")]
pub struct AsyncTmp108<I2C: AsyncI2c> {
    inner: Inner<AsyncInterface<I2C>>,
    addr: u8,
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Create a new TMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::{A0, Tmp108};
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new(i2c, A0::Sda);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = Interface::new(i2c, a0);
        let addr = interface.addr;
        let inner = Inner::new(interface);

        Self { inner, addr }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_vplus(i2c);
    /// assert_eq!(tmp.addr(), 0x49);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_sda(i2c);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_scl(i2c);
    /// assert_eq!(tmp.addr(), 0x4b);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut i2c = tmp.destroy();
    /// i2c.done();
    /// ```
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncTmp108<I2C> {
    /// Create a new TMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::{A0, AsyncTmp108};
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new(i2c, A0::Sda);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = AsyncInterface::new(i2c, a0);
        let addr = interface.addr;
        let inner = Inner::new(interface);

        Self { inner, addr }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_vplus(i2c);
    /// assert_eq!(tmp.addr(), 0x49);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_sda(i2c);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_scl(i2c);
    /// assert_eq!(tmp.addr(), 0x4b);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let mut i2c = tmp.destroy();
    /// i2c.done();
    /// # });
    /// ```
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }

    /// Create a new [`AlertTmp108`] instance by consuming the original
    /// `AsyncTmp108` instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let alert_tmp = tmp.into_alert(alert);
    /// let (mut i2c, mut alert) = alert_tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # });
    /// ```
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, ALERT> {
        AlertTmp108 { tmp108: self, alert }
    }
}

/// Async TMP108 driver with an ALERT GPIO pin attached.
///
/// Wraps a bare [`Tmp108`] with a pin implementing
/// [`embedded_hal_async::digital::Wait`] (and
/// [`embedded_hal::digital::InputPin`]), so it can implement
/// [`embedded_sensors_hal_async::temperature::TemperatureThresholdWait`].
///
/// # Notes on alert behavior
///
/// The driver's [`wait_for_temperature_threshold`][1] implementation
/// relies on the `embedded-hal-async` [`Wait`][2] trait contract:
/// implementations must report transitions that occur between `Wait`
/// calls (e.g. via a pending-edge / latched-interrupt mechanism in the
/// MCU's GPIO controller). If the HAL implementation drops pending
/// edges, the driver will miss them — this is a property of the HAL,
/// not the driver.
///
/// The chip's `Polarity` (active-low vs active-high) is read from the
/// configuration register on every call to `wait_for_temperature_threshold`.
/// Do **not** reconfigure polarity while a `wait_for_temperature_threshold`
/// future is pending — the awaiting future will continue to wait for the
/// old polarity while the chip's ALERT output follows the new one.
///
/// In Comparator mode the ALERT pin remains asserted as long as the
/// temperature is outside the `[TLow + HYS, THigh − HYS]` band; calling
/// `wait_for_temperature_threshold` in a tight loop while the chip is
/// still over-temperature will return immediately on every iteration
/// (because [`wait_for_low`][3] / [`wait_for_high`][4] return
/// immediately when the pin is already at the requested level). For
/// repeated-trigger workflows prefer Interrupt mode, or apply
/// application-level backoff between iterations.
///
/// [1]: embedded_sensors_hal_async::temperature::TemperatureThresholdWait::wait_for_temperature_threshold
/// [2]: embedded_hal_async::digital::Wait
/// [3]: embedded_hal_async::digital::Wait::wait_for_low
/// [4]: embedded_hal_async::digital::Wait::wait_for_high
#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
pub struct AlertTmp108<
    I2C: embedded_hal_async::i2c::I2c,
    ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
> {
    tmp108: AsyncTmp108<I2C>,
    alert: ALERT,
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    AlertTmp108<I2C, ALERT>
{
    /// Create a new ALERTTMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::{A0, AlertTmp108};
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new(i2c, A0::Sda, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x4a);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new(i2c: I2C, a0: A0, alert: ALERT) -> Self {
        let tmp108 = AsyncTmp108::new(i2c, a0);
        Self { tmp108, alert }
    }

    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x48);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Gnd, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to V+, resulting in an
    /// instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_vplus(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x49);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Vplus, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SDA, resulting in an
    /// instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_sda(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x4a);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_sda(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Sda, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SCL, resulting in an
    /// instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_scl(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x4b);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_scl(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Scl, alert)
    }

    /// Borrow the inner [`AsyncTmp108`] for sensor operations
    /// (read temperature, configure, set thresholds…). Useful when you
    /// want to call sensor methods directly, without going through the
    /// alert-aware threshold trait.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x48);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn sensor(&self) -> &AsyncTmp108<I2C> {
        &self.tmp108
    }

    /// Mutably borrow the inner [`AsyncTmp108`] for sensor operations.
    /// See [`sensor`][Self::sensor].
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AlertTmp108, Config};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let alert = digital::Mock::new(&[]);
    /// let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let cfg = tmp.sensor_mut().read_configuration().await.unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn sensor_mut(&mut self) -> &mut AsyncTmp108<I2C> {
        &mut self.tmp108
    }

    /// Destructure the wrapper back into its [`AsyncTmp108`] sensor and
    /// ALERT pin halves. The inverse of [`AsyncTmp108::into_alert`].
    ///
    /// Unlike [`destroy`][Self::destroy] (which returns the raw I2C
    /// bus, dropping the sensor's typed wrapper), this preserves the
    /// sensor's state so the caller can continue using it directly.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let (sensor, alert) = tmp.into_inner();
    /// // The sensor still owns the I2C bus.
    /// let mut i2c = sensor.destroy();
    /// i2c.done();
    /// let mut alert = alert;
    /// alert.done();
    /// # });
    /// ```
    pub fn into_inner(self) -> (AsyncTmp108<I2C>, ALERT) {
        (self.tmp108, self.alert)
    }

    /// Destroy the driver instance, return the I2C bus instance and ALERT pin instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let (mut i2c, mut alert) = tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # });
    /// ```
    pub fn destroy(self) -> (I2C, ALERT) {
        (self.tmp108.destroy(), self.alert)
    }
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Probe the chip's presence by reading the configuration register.
    ///
    /// The TMP108 does not expose a `WHO_AM_I` / device-ID register, so a
    /// true identity probe is impossible. This method does the next-best
    /// thing: it reads the configuration register and reports whether
    /// the value matches the chip's documented power-on reset (POR)
    /// value `0x1022`. Useful immediately after power-on to confirm the
    /// chip is freshly out of reset and on the bus.
    ///
    /// # Returns
    ///
    /// - `Ok(true)` — the read succeeded and the configuration register
    ///   matches the POR value. Strong evidence the chip is present and
    ///   has not yet been reconfigured.
    /// - `Ok(false)` — the read succeeded but the configuration differs
    ///   from POR. Still strong evidence the chip is present (it `ACKed`
    ///   and returned plausible register data) but it was already
    ///   reconfigured since power-on. False negatives are unavoidable on
    ///   any boot path where the chip was configured before this method
    ///   ran.
    /// - `Err(_)` — the I2C read failed. Most likely cause is that no
    ///   chip is present at the expected address (NACK), but any bus
    ///   error reports here as well.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// // Chip returns the POR configuration -> probe() reports true.
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert!(tmp.probe().unwrap());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn probe(&mut self) -> Result<bool, I2C::Error> {
        let raw = self.inner.configuration().read()?;
        Ok(u16::from_le_bytes(raw.into()) == ops::POR_CONFIG)
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Config, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let cfg = tmp.read_configuration().unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        let c = self.inner.configuration().read()?;
        Ok(ops::decode_config(c))
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.configure(Config {
    ///     thermostat_mode: Thermostat::Interrupt,
    ///     alert_polarity: Polarity::ActiveHigh,
    ///     conversion_rate: ConversionRate::SixteenHz,
    ///     hysteresis: Hysteresis::FourC,
    /// }).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| ops::apply_config(r, config))
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let temp = tmp.temperature().unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn temperature(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.temperature().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x21, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.one_shot().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::OneShot))
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.shutdown().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::Shutdown))
    }

    /// Wait one conversion period, then read the temperature register.
    ///
    /// Reads the configuration register to discover the current
    /// [`ConversionRate`], delays for one period (1/CR), and then reads
    /// the temperature register. Intended for callers driving the chip
    /// in [`Mode::Continuous`] who want to align reads with the chip's
    /// conversion cadence.
    ///
    /// # Stale-reading on first call
    ///
    /// The TMP108's conversion period (1/CR — 4 s, 1 s, 250 ms, or
    /// 62.5 ms) is **not** the same as its conversion **time** (~30 ms
    /// regardless of CR). After entering [`Mode::Continuous`] the chip's
    /// next conversion is not phase-aligned with when you enabled it,
    /// so the first call to this method may return the previous
    /// conversion result. For "guaranteed fresh" semantics, use
    /// [`one_shot`][Self::one_shot] followed by a delay of one period
    /// and a [`temperature`][Self::temperature] read, or discard the
    /// first reading after entering Continuous.
    ///
    /// # I²C cost per call
    ///
    /// Each call performs **two** I²C transactions: a configuration
    /// read (to determine the CR) and a temperature read. Callers in
    /// power- or bandwidth-sensitive loops who know they will not
    /// change CR can avoid the per-call configuration read by calling
    /// [`read_configuration`][Self::read_configuration] once, computing
    /// the period delay themselves, and calling
    /// [`temperature`][Self::temperature] directly.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when either the configuration read or the
    /// temperature read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::delay::NoopDelay;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut delay = NoopDelay::new();
    /// let temp = tmp.wait_for_temperature(&mut delay).unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<Celsius, I2C::Error> {
        let config = self.read_configuration()?;
        delay.delay_us(conversion_period_us(config.conversion_rate));
        self.temperature()
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.low_limit().unwrap();
    /// assert_eq!(limit.to_degrees(), 25.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn low_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_low().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature low limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Celsius, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_low_limit(Celsius::try_from_degrees(25.0).unwrap()).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn set_low_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner.t_low().write(|r| *r = TLow::from(limit.to_register()))
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.high_limit().unwrap();
    /// assert_eq!(limit.to_degrees(), 80.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn high_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_high().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature high limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Celsius, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_high_limit(Celsius::try_from_degrees(80.0).unwrap()).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn set_high_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner.t_high().write(|r| *r = THigh::from(limit.to_register()))
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncTmp108<I2C> {
    /// Probe the chip's presence by reading the configuration register.
    ///
    /// See [`Tmp108::probe`] for full semantics. The async flavor has
    /// the same `Ok(true)` / `Ok(false)` / `Err(_)` contract.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// assert!(tmp.probe().await.unwrap());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn probe(&mut self) -> Result<bool, I2C::Error> {
        let raw = self.inner.configuration().read_async().await?;
        Ok(u16::from_le_bytes(raw.into()) == ops::POR_CONFIG)
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Config};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let cfg = tmp.read_configuration().await.unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        let c = self.inner.configuration().read_async().await?;
        Ok(ops::decode_config(c))
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Config, ConversionRate, Hysteresis, Polarity, Thermostat};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.configure(Config {
    ///     thermostat_mode: Thermostat::Interrupt,
    ///     alert_polarity: Polarity::ActiveHigh,
    ///     conversion_rate: ConversionRate::SixteenHz,
    ///     hysteresis: Hysteresis::FourC,
    /// }).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| ops::apply_config(r, config))
            .await
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let temp = tmp.temperature().await.unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn temperature(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.temperature().read_async().await?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x21, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.one_shot().await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::OneShot))
            .await
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.shutdown().await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Shutdown))
            .await
    }

    /// Initiate continuous conversions.
    ///
    /// Switches the chip into [`Mode::Continuous`], runs the user-supplied
    /// closure, and unconditionally returns the chip to [`Mode::Shutdown`]
    /// before returning, **regardless of whether the closure succeeded or
    /// failed**. This ensures the chip is not left burning current after
    /// a transient bus failure inside the closure.
    ///
    /// # Cancel-safety
    ///
    /// **The returned future is *not* cancel-safe.** If it is dropped
    /// before completion (e.g. by `embassy_futures::select!`,
    /// `tokio::time::timeout`, or a task cancellation), the chip is left
    /// in [`Mode::Continuous`] and will continue to draw current
    /// indefinitely. Callers that need cancellation must structure their
    /// own recovery, for example by calling
    /// [`shutdown`][Self::shutdown] after a cancelled call.
    ///
    /// # Errors
    ///
    /// - If the closure returns `Err(e)`, the cleanup `shutdown()` still
    ///   runs but its result is discarded; the closure's error is
    ///   returned.
    /// - If the closure returns `Ok(())` and the cleanup `shutdown()`
    ///   fails, that I2C error is returned.
    /// - If the initial transition into `Mode::Continuous` fails, the
    ///   closure is not invoked and the I2C error is returned.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     // Enter continuous: read cfg, modify M bits, write back.
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
    ///     // Inside the closure: read temperature once.
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    ///     // continuous() returns the chip to shutdown on exit.
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.continuous(async |t| {
    ///     let _ = t.temperature().await?;
    ///     Ok(())
    /// }).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn continuous<F>(&mut self, f: F) -> Result<(), I2C::Error>
    where
        F: AsyncFnOnce(&mut Self) -> Result<(), I2C::Error>,
    {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Continuous))
            .await?;

        // Run the user closure and capture its result, but always attempt
        // shutdown afterwards so the chip is not left in Continuous mode.
        // The closure's error takes precedence over a shutdown failure:
        // the user's failure is the actionable signal, the cleanup error
        // is a secondary symptom.
        let user_result = f(self).await;
        let cleanup_result = self.shutdown().await;
        user_result.and(cleanup_result)
    }

    /// Wait one conversion period, then read the temperature register.
    ///
    /// See [`Tmp108::wait_for_temperature`] for the full semantics,
    /// including the stale-first-reading and per-call I²C cost notes.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when either the configuration read or the
    /// temperature read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::delay::NoopDelay;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let mut delay = NoopDelay::new();
    /// let temp = tmp.wait_for_temperature(&mut delay).await.unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn wait_for_temperature<DELAY: AsyncDelayNs>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<Celsius, I2C::Error> {
        let config = self.read_configuration().await?;
        delay.delay_us(conversion_period_us(config.conversion_rate)).await;
        self.temperature().await
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.low_limit().await.unwrap();
    /// assert_eq!(limit.to_degrees(), 25.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn low_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_low().read_async().await?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature low limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Celsius};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.set_low_limit(Celsius::try_from_degrees(25.0).unwrap()).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn set_low_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner
            .t_low()
            .write_async(|r| *r = TLow::from(limit.to_register()))
            .await
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.high_limit().await.unwrap();
    /// assert_eq!(limit.to_degrees(), 80.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn high_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_high().read_async().await?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature high limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Celsius};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.set_high_limit(Celsius::try_from_degrees(80.0).unwrap()).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn set_high_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner
            .t_high()
            .write_async(|r| *r = THigh::from(limit.to_register()))
            .await
    }
}

/// Compute the chip's conversion period (1/CR) in microseconds.
const fn conversion_period_us(rate: ConversionRate) -> u32 {
    match rate {
        ConversionRate::QuarterHz => 4_000_000,
        ConversionRate::OneHz => 1_000_000,
        ConversionRate::FourHz => 250_000,
        ConversionRate::SixteenHz => 62_500,
    }
}

/// Blocking-side I²C wire interface for [`Tmp108`].
///
/// Wraps an [`embedded_hal::i2c::I2c`] and the chip address; implements
/// [`device_driver::RegisterInterface`] so the `device-driver` codegen
/// can drive it.
struct Interface<I2C: I2c> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Interface<I2C> {
    fn new(i2c: I2C, a0: A0) -> Self {
        Self { i2c, addr: a0.into() }
    }
}

impl<I2C: I2c> RegisterInterfaceBase for Interface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

impl<I2C: I2c> RegisterInterface for Interface<I2C> {
    fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        debug_assert_eq!(data.len(), 2, "TMP108 registers are 16-bit");
        let mut buf = [0; 3];
        buf[0] = address;
        buf[1..].copy_from_slice(data);
        self.i2c.write(self.addr, &buf)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[address], data)
    }
}

/// Async-side I²C wire interface for [`AsyncTmp108`].
///
/// Wraps an [`embedded_hal_async::i2c::I2c`] and the chip address;
/// implements [`device_driver::AsyncRegisterInterface`] so the
/// `device-driver` codegen can drive it asynchronously.
#[cfg(feature = "async")]
struct AsyncInterface<I2C: AsyncI2c> {
    i2c: I2C,
    addr: u8,
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncInterface<I2C> {
    fn new(i2c: I2C, a0: A0) -> Self {
        Self { i2c, addr: a0.into() }
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> RegisterInterfaceBase for AsyncInterface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncRegisterInterface for AsyncInterface<I2C> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        debug_assert_eq!(data.len(), 2, "TMP108 registers are 16-bit");
        let mut buf = [0; 3];
        buf[0] = address;
        buf[1..].copy_from_slice(data);
        self.i2c.write(self.addr, &buf).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[address], data).await
    }
}

/// Tmp108 Errors
///
/// The `E` parameter is the underlying I2C error type. The `P` parameter
/// is the error type of an optional ALERT GPIO pin; it defaults to
/// [`core::convert::Infallible`] so bare [`Tmp108`] (which has no pin)
/// uses `Error<I2C::Error>` and never produces a [`Pin`][Self::Pin]
/// error. [`AlertTmp108`] specializes to
/// `Error<I2C::Error, ALERT::Error>` and uses the [`Pin`][Self::Pin]
/// variant when the GPIO peripheral fails.
///
/// `Error` implements [`Clone`], [`Copy`], [`PartialEq`], and [`Eq`]
/// when both `E` and `P` do. [`Debug`] is always available because
/// `embedded_hal::i2c::Error` and `embedded_hal::digital::Error` both
/// require it.
#[derive(Debug)]
pub enum Error<E: embedded_hal::i2c::Error, P: embedded_hal::digital::Error = core::convert::Infallible> {
    /// I2C bus error.
    Bus(E),
    /// Input failed validation (out of range, NaN, ±∞, unsupported value).
    InvalidInput,
    /// ALERT pin GPIO error.
    Pin(P),
}

// Manual Clone (rather than #[derive]) because the auto-derive would
// emit `where E: Clone, P: Clone, E: embedded_hal::i2c::Error,
// P: embedded_hal::digital::Error` and we want only the first two
// bounds — the trait bounds are already implied by the struct.
// `expl_impl_clone_on_copy` would prefer #[derive], but doing so would
// require the same fix and the manual form is clearer about the bounds.
#[allow(clippy::expl_impl_clone_on_copy)]
impl<E: embedded_hal::i2c::Error + Clone, P: embedded_hal::digital::Error + Clone> Clone for Error<E, P> {
    fn clone(&self) -> Self {
        match self {
            Self::Bus(e) => Self::Bus(e.clone()),
            Self::InvalidInput => Self::InvalidInput,
            Self::Pin(e) => Self::Pin(e.clone()),
        }
    }
}

impl<E: embedded_hal::i2c::Error + Copy, P: embedded_hal::digital::Error + Copy> Copy for Error<E, P> {}

impl<E: embedded_hal::i2c::Error + PartialEq, P: embedded_hal::digital::Error + PartialEq> PartialEq for Error<E, P> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Bus(a), Self::Bus(b)) => a == b,
            (Self::InvalidInput, Self::InvalidInput) => true,
            (Self::Pin(a), Self::Pin(b)) => a == b,
            _ => false,
        }
    }
}

impl<E: embedded_hal::i2c::Error + Eq, P: embedded_hal::digital::Error + Eq> Eq for Error<E, P> {}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<E: embedded_hal::i2c::Error, P: embedded_hal::digital::Error> embedded_sensors_hal::sensor::Error for Error<E, P> {
    fn kind(&self) -> embedded_sensors_hal::sensor::ErrorKind {
        embedded_sensors_hal::sensor::ErrorKind::Other
    }
}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<I2C: embedded_hal::i2c::I2c> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C> {
    type Error = Error<I2C::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<I2C: embedded_hal::i2c::I2c> embedded_sensors_hal::temperature::TemperatureSensor for Tmp108<I2C> {
    fn temperature(&mut self) -> Result<embedded_sensors_hal::temperature::DegreesCelsius, Self::Error> {
        self.temperature().map(Celsius::to_degrees).map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<E: embedded_hal_async::i2c::Error, P: embedded_hal::digital::Error> embedded_sensors_hal_async::sensor::Error
    for Error<E, P>
{
    fn kind(&self) -> embedded_sensors_hal_async::sensor::ErrorKind {
        embedded_sensors_hal_async::sensor::ErrorKind::Other
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::sensor::ErrorType for AsyncTmp108<I2C> {
    type Error = Error<I2C::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureSensor
    for AsyncTmp108<I2C>
{
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.temperature().await.map(Celsius::to_degrees).map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::sensor::ErrorType for AlertTmp108<I2C, ALERT>
{
    type Error = Error<I2C::Error, <ALERT as embedded_hal::digital::ErrorType>::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureSensor for AlertTmp108<I2C, ALERT>
{
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.tmp108
            .temperature()
            .await
            .map(Celsius::to_degrees)
            .map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureThresholdSet
    for AsyncTmp108<I2C>
{
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        // The trait takes a continuous f32; the chip takes a 12-bit
        // fixed-point value. Parse at the boundary and reject anything
        // the part cannot hold.
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.set_low_limit(limit).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.set_high_limit(limit).await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureThresholdSet for AlertTmp108<I2C, ALERT>
{
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.tmp108.set_low_limit(limit).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.tmp108.set_high_limit(limit).await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureThresholdWait for AlertTmp108<I2C, ALERT>
{
    async fn wait_for_temperature_threshold(
        &mut self,
    ) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        let config = self.tmp108.read_configuration().await.map_err(Error::Bus)?;

        match (config.thermostat_mode, config.alert_polarity) {
            // In comparator mode, the ALERT pin remains active even after triggering.
            //
            // If called in a loop, next iteration would return immediately (after reading config
            // again) if temperature remains outside threshold.
            //
            // ALERT pin only resets when temperature falls within the range of (Tlow + HYS) and
            // (Thigh - HYS).
            (Thermostat::Comparator, Polarity::ActiveLow) => {
                self.alert.wait_for_low().await.map_err(Error::Pin)?;
            }
            (Thermostat::Comparator, Polarity::ActiveHigh) => {
                self.alert.wait_for_high().await.map_err(Error::Pin)?;
            }

            // In interrupt mode, the ALERT pin is immediately reset (by reading config register)
            // after triggering.
            //
            // If called in a loop, next iteration would wait even if temperature remains outside
            // threshold.
            (Thermostat::Interrupt, Polarity::ActiveLow) => {
                self.alert.wait_for_falling_edge().await.map_err(Error::Pin)?;
                let _ = self.tmp108.read_configuration().await.map_err(Error::Bus)?;
            }
            (Thermostat::Interrupt, Polarity::ActiveHigh) => {
                self.alert.wait_for_rising_edge().await.map_err(Error::Pin)?;
                let _ = self.tmp108.read_configuration().await.map_err(Error::Bus)?;
            }
        }

        // Return temperature at time of trigger for caller to determine which threshold was crossed.
        let temperature = self.tmp108.temperature().await.map_err(Error::Bus)?;
        Ok(temperature.to_degrees())
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureHysteresis
    for AsyncTmp108<I2C>
{
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        // The trait method takes a continuous range of f32 °C values, but
        // the chip only supports four discrete hysteresis settings:
        // 0, 1, 2, and 4 °C. ops::snap_hysteresis snaps within a 0.05 °C
        // tolerance band; anything outside the band (or non-finite) is
        // rejected with `Error::InvalidInput`.
        let snapped = ops::snap_hysteresis(hysteresis).ok_or(Error::InvalidInput)?;

        let mut config = self.read_configuration().await.map_err(Error::Bus)?;
        config.hysteresis = snapped;
        self.configure(config).await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureHysteresis for AlertTmp108<I2C, ALERT>
{
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108
            .set_temperature_threshold_hysteresis(hysteresis)
            .await
            .map_err(widen_pin_err)
    }
}

/// Widen an `Error<E>` (with Pin = Infallible) to `Error<E, P>` for any
/// `P`. Used by the `AlertTmp108` trait impls that delegate to bare
/// [`AsyncTmp108`] methods (which cannot themselves produce a `Pin`
/// error).
#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
fn widen_pin_err<E: embedded_hal_async::i2c::Error, P: embedded_hal::digital::Error>(
    e: Error<E, core::convert::Infallible>,
) -> Error<E, P> {
    match e {
        Error::Bus(e) => Error::Bus(e),
        Error::InvalidInput => Error::InvalidInput,
        Error::Pin(never) => match never {},
    }
}

#[cfg(test)]
mod tests {
    use super::inner::Configuration;
    use super::*;

    /// A `Configuration` initialized to the chip's power-on reset
    /// value.
    ///
    /// In device-driver 2.x a fieldset's `Default` is all-zeroes; the
    /// documented reset value is carried by the *register operation*
    /// instead. This helper reads it back out of the generated
    /// register operation so the tests below stay pinned to the DDSL
    /// manifest rather than to a hand-written constant.
    fn por_configuration() -> Configuration {
        let mut tmp = Tmp108::new_with_a0_gnd(embedded_hal_mock::eh1::i2c::Mock::new(&[]));
        let cfg = tmp.inner.configuration().reset_value();
        let mut i2c = tmp.destroy();
        i2c.done();
        cfg
    }

    #[test]
    fn default_configuration() {
        let cfg = por_configuration();
        assert_eq!(u16::from_le_bytes(cfg.into()), 0x1022);
    }

    #[test]
    fn modify_mode() {
        let mut cfg = por_configuration();
        cfg.set_m(Mode::Shutdown);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1020);
        cfg.set_m(Mode::OneShot);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1021);
        cfg.set_m(Mode::Continuous);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
    }

    #[test]
    fn modify_thermostat_mode() {
        let mut cfg = por_configuration();
        cfg.set_tm(Thermostat::Comparator);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_tm(Thermostat::Interrupt);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1026);
    }

    #[test]
    fn modify_watchdog_temperature_flags() {
        let mut cfg = por_configuration();
        cfg.set_fl(true);
        cfg.set_fh(false);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x102a);
        cfg.set_fl(false);
        cfg.set_fh(true);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1032);
        cfg.set_fl(true);
        cfg.set_fh(true);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x103a);
    }

    #[test]
    fn modify_conversion_rate() {
        let mut cfg = por_configuration();
        cfg.set_cr(ConversionRate::QuarterHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1002);
        cfg.set_cr(ConversionRate::OneHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_cr(ConversionRate::FourHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1042);
        cfg.set_cr(ConversionRate::SixteenHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1062);
    }

    #[test]
    fn modify_hysteresis() {
        let mut cfg = por_configuration();
        cfg.set_hys(Hysteresis::ZeroC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x0022);
        cfg.set_hys(Hysteresis::OneC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_hys(Hysteresis::TwoC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x2022);
        cfg.set_hys(Hysteresis::FourC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x3022);
    }

    #[test]
    fn modify_polarity() {
        let mut cfg = por_configuration();
        cfg.set_pol(Polarity::ActiveLow);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_pol(Polarity::ActiveHigh);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x9022);
    }

    mod ops_tests {
        use super::*;

        /// Exhaustive tests for the [`Celsius`] newtype and its
        /// register codec.
        mod celsius {
            // Exactness is the property under test. The scale factor is a
            // power of two and every value fits an `f32` mantissa, so these
            // conversions are lossless. Comparing approximately would weaken
            // the assertions, not strengthen them.
            #![allow(clippy::float_cmp)]

            use super::*;
            use crate::ops::{HIGHEST_ACCEPTED, LOWEST_ACCEPTED, MAX_SIXTEENTHS, MIN_SIXTEENTHS};

            /// Every temperature the type can hold. 4096 of them.
            fn all() -> impl Iterator<Item = Celsius> {
                (MIN_SIXTEENTHS..=MAX_SIXTEENTHS)
                    .map(|s| Celsius::from_sixteenths(s).expect("in range by construction"))
            }

            #[test]
            fn decoding_a_register_is_total() {
                // All 65,536 bit patterns, not a sample of them.
                for word in 0..=u16::MAX {
                    let c = Celsius::from_register(word.to_be_bytes());
                    assert!(
                        (MIN_SIXTEENTHS..=MAX_SIXTEENTHS).contains(&c.sixteenths()),
                        "word {word:#06x} decoded out of range: {c:?}"
                    );
                }
            }

            #[test]
            fn every_temperature_roundtrips_through_a_register() {
                for c in all() {
                    assert_eq!(Celsius::from_register(c.to_register()), c);
                }
            }

            #[test]
            fn every_temperature_roundtrips_through_sixteenths() {
                for c in all() {
                    assert_eq!(Celsius::from_sixteenths(c.sixteenths()), Ok(c));
                }
            }

            #[test]
            fn every_temperature_roundtrips_through_degrees() {
                for c in all() {
                    assert_eq!(Celsius::try_from_degrees(c.to_degrees()), Ok(c));
                }
            }

            #[test]
            fn sixteenths_outside_the_range_are_rejected() {
                assert_eq!(Celsius::from_sixteenths(MIN_SIXTEENTHS - 1), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::from_sixteenths(MAX_SIXTEENTHS + 1), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::from_sixteenths(i16::MIN), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::from_sixteenths(i16::MAX), Err(OutOfRange::TooHigh));
            }

            #[test]
            fn degrees_the_part_cannot_hold_are_rejected() {
                // The values that silently became 0 C, 127.9375 C and -128 C before.
                assert_eq!(Celsius::try_from_degrees(f32::NAN), Err(OutOfRange::NotANumber));
                assert_eq!(Celsius::try_from_degrees(f32::INFINITY), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::try_from_degrees(f32::NEG_INFINITY), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(1000.0), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::try_from_degrees(-500.0), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(128.0), Err(OutOfRange::TooHigh));
            }

            #[test]
            fn the_extremes_are_exactly_representable() {
                assert_eq!(Celsius::MIN.to_degrees(), -128.0);
                assert_eq!(Celsius::MAX.to_degrees(), 127.9375);
                assert_eq!(Celsius::ZERO.to_degrees(), 0.0);

                // One LSB short of +128, not +128. Two's complement is lopsided.
                assert_eq!(Celsius::try_from_degrees(127.9375), Ok(Celsius::MAX));
                assert_eq!(Celsius::try_from_degrees(-128.0), Ok(Celsius::MIN));
            }

            #[test]
            fn rounding_bounds_bracket_the_representable_range() {
                // The accepted interval is open and sits exactly half an LSB
                // outside the representable range at each end.
                assert_eq!(LOWEST_ACCEPTED, f32::from(MIN_SIXTEENTHS) - 0.5);
                assert_eq!(HIGHEST_ACCEPTED, f32::from(MAX_SIXTEENTHS) + 0.5);

                // Just inside rounds back to the endpoint; exactly on the
                // bound does not.
                assert_eq!(Celsius::try_from_degrees(-128.03).unwrap(), Celsius::MIN);
                assert_eq!(Celsius::try_from_degrees(-128.04), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(127.96).unwrap(), Celsius::MAX);
                assert_eq!(Celsius::try_from_degrees(127.97), Err(OutOfRange::TooHigh));
            }

            #[test]
            fn degrees_round_to_the_nearest_sixteenth() {
                // 0.0625 C per LSB, so 0.03 rounds down to 0 and 0.04 rounds up to 1.
                assert_eq!(Celsius::try_from_degrees(0.03).unwrap().sixteenths(), 0);
                assert_eq!(Celsius::try_from_degrees(0.04).unwrap().sixteenths(), 1);
                assert_eq!(Celsius::try_from_degrees(-0.03).unwrap().sixteenths(), 0);
                assert_eq!(Celsius::try_from_degrees(-0.04).unwrap().sixteenths(), -1);
            }

            #[test]
            fn known_datasheet_values_decode() {
                // Table 7 of the datasheet, as register words.
                for (word, degrees) in [
                    (0x7ff0_u16, 127.9375_f32),
                    (0x6400, 100.0),
                    (0x5000, 80.0),
                    (0x3200, 50.0),
                    (0x1900, 25.0),
                    (0x0040, 0.25),
                    (0x0000, 0.0),
                    (0xffc0, -0.25),
                    (0xe700, -25.0),
                    (0xc900, -55.0),
                    (0x8000, -128.0),
                ] {
                    let c = Celsius::from_register(word.to_be_bytes());
                    assert_eq!(c.to_degrees(), degrees, "word {word:#06x}");
                    assert_eq!(c.to_register(), word.to_be_bytes(), "word {word:#06x}");
                }
            }

            #[test]
            fn unused_low_bits_are_discarded_not_truncated_toward_zero() {
                // Per the datasheet (Table 6 / Table 12) bits 3..0 of the low
                // byte are hardwired zero and "always read 0". `from_register`
                // is nonetheless total and must floor rather than truncate
                // toward zero for negatives.
                assert_eq!(Celsius::from_register(0xffff_u16.to_be_bytes()).sixteenths(), -1);
                assert_eq!(Celsius::from_register(0x0001_u16.to_be_bytes()).sixteenths(), 0);

                // The power-on T_HIGH value, which the datasheet quotes as
                // 0x7FF8 (+127.9375 C) — it carries a stray bit 3 that the
                // register cannot store once written.
                assert_eq!(Celsius::from_register(0x7ff8_u16.to_be_bytes()), Celsius::MAX);
            }
        }

        #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
        #[test]
        fn snap_hysteresis_accepts_within_tolerance() {
            let cases: &[(f32, Hysteresis)] = &[
                (0.0, Hysteresis::ZeroC),
                (1.0, Hysteresis::OneC),
                (2.0, Hysteresis::TwoC),
                (4.0, Hysteresis::FourC),
                (0.04, Hysteresis::ZeroC),
                (0.1_f32 + 0.9_f32, Hysteresis::OneC),
                (1.95, Hysteresis::TwoC),
                (3.97, Hysteresis::FourC),
            ];
            for (input, expected) in cases {
                assert_eq!(
                    ops::snap_hysteresis(*input),
                    Some(*expected),
                    "input {input} should snap to {expected:?}"
                );
            }
        }

        #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
        #[test]
        fn snap_hysteresis_rejects_out_of_tolerance_and_non_finite() {
            for bad in [-0.5_f32, 0.5, 3.0, 5.0, -1.0, 10.0] {
                assert_eq!(ops::snap_hysteresis(bad), None);
            }
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert_eq!(ops::snap_hysteresis(bad), None);
            }
        }

        #[test]
        fn decode_apply_config_roundtrip() {
            // For every non-default Config we set, applying it to the POR
            // configuration and reading it back must yield the same Config
            // (modulo bits we don't expose: M, FL, FH, ID).
            let cfg = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::SixteenHz,
                hysteresis: Hysteresis::FourC,
            };

            let mut reg = por_configuration();
            ops::apply_config(&mut reg, cfg);
            assert_eq!(ops::decode_config(reg), cfg);
        }

        #[test]
        fn por_config_matches_default_configuration() {
            // ops::POR_CONFIG must match the chip's documented POR value
            // (0x1022) and the generated configuration register's reset
            // value. If the DDSL manifest changes the reset value,
            // probe()'s contract changes too — this test pins it.
            let cfg = por_configuration();
            assert_eq!(u16::from_le_bytes(cfg.into()), ops::POR_CONFIG);
        }

        #[test]
        fn config_derives_eq() {
            fn assert_eq_trait<T: Eq>(_: &T) {}

            // Config is now `Eq` (and `PartialEq` and `Hash`); use the
            // stronger trait so we know it actually compiles.
            let a = Config::default();
            let b = Config::default();
            assert_eq_trait(&a);
            assert_eq!(a, b);
        }

        #[test]
        fn error_derives_with_eq_kind() {
            // Error<E, P> implements Clone/Copy/PartialEq/Eq when both
            // E and P do. Verify with concrete types that satisfy those
            // bounds (ErrorKind from embedded-hal is the canonical
            // small witness here).
            type EK = embedded_hal::i2c::ErrorKind;
            type PK = embedded_hal::digital::ErrorKind;

            fn assert_traits<T: Clone + Copy + PartialEq + Eq + core::fmt::Debug>(_: &T) {}

            let invalid_a: Error<EK, PK> = Error::InvalidInput;
            let invalid_b: Error<EK, PK> = Error::InvalidInput;
            assert_traits(&invalid_a);
            assert_eq!(invalid_a, invalid_b);
            let invalid_c = invalid_a; // Copy

            assert_eq!(invalid_a, invalid_c);
            assert_eq!(invalid_a.clone(), invalid_b);

            let bus_err: Error<EK, PK> = Error::Bus(EK::Other);
            let pin_err: Error<EK, PK> = Error::Pin(PK::Other);
            assert_ne!(bus_err, pin_err);
            assert_ne!(bus_err, invalid_a);
        }
    }

    #[cfg(not(feature = "async"))]
    mod blocking {
        use assert_approx_eq::assert_approx_eq;
        use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

        use super::*;

        #[test]
        fn handle_a0_pin_accordingly() {
            let expectations = vec![];

            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_gnd(mock);
            assert_eq!(tmp.addr(), 0x48);
            let mut mock = tmp.destroy();
            mock.done();
            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_vplus(mock);
            assert_eq!(tmp.addr(), 0x49);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_sda(mock);
            assert_eq!(tmp.addr(), 0x4a);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_scl(mock);
            assert_eq!(tmp.addr(), 0x4b);
            let mut mock = tmp.destroy();
            mock.done();
        }

        #[test]
        fn change_configuration() {
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
                Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
            ];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
            let result = tmp108.read_configuration();
            assert!(result.is_ok());

            let config = result.unwrap();
            assert_eq!(config, Config::default());

            let config = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::SixteenHz,
                hysteresis: Hysteresis::FourC,
            };

            let result = tmp108.configure(config);
            assert!(result.is_ok());

            let result = tmp108.read_configuration();
            assert!(result.is_ok());

            let new_config = result.unwrap();
            assert_eq!(config, new_config);

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn read_temperature_default_address() {
            let expectations = vec![
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x7f, 0xf0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x64, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x4b, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x40])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xff, 0xc0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xe7, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xc9, 0x00])],
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            for (e, t) in expectations.iter().zip(temps.iter()) {
                let mock = Mock::new(e);
                let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
                let result = tmp108.temperature();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);

                let mut mock = tmp108.destroy();
                mock.done();
            }
        }

        #[test]
        fn set_and_read_low_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x02, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x02], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x02, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x02], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x02, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_low_limit(limit);
                assert!(result.is_ok());

                let result = tmp108.low_limit();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn set_and_read_high_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x03, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x03], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x03, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x03], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x03, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_high_limit(limit);
                assert!(result.is_ok());

                let result = tmp108.high_limit();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn probe_returns_true_for_por_value() {
            // Configuration register at 0x01 returns the POR value 0x1022.
            // The register layout is little-endian per tmp108.toml so the
            // wire bytes are [0x22, 0x10].
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe(), Ok(true));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn probe_returns_false_for_non_por_value() {
            // Chip is present (ACKs) but has been reconfigured.
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe(), Ok(false));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn probe_propagates_bus_error() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]).with_error(
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            )];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert!(tmp108.probe().is_err());

            let mut mock = tmp108.destroy();
            mock.done();
        }
    }

    #[cfg(feature = "async")]
    mod asynchronous {
        use assert_approx_eq::assert_approx_eq;
        use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

        use super::*;

        #[tokio::test]
        async fn handle_a0_pin_accordingly() {
            let expectations = vec![];

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_gnd(mock);
            assert_eq!(tmp.addr(), 0x48);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_vplus(mock);
            assert_eq!(tmp.addr(), 0x49);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_sda(mock);
            assert_eq!(tmp.addr(), 0x4a);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_scl(mock);
            assert_eq!(tmp.addr(), 0x4b);
            let mut mock = tmp.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn change_configuration() {
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
                Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
            ];

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);
            let result = tmp108.read_configuration().await;
            assert!(result.is_ok());

            let config = result.unwrap();
            assert_eq!(config, Config::default());

            let config = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::SixteenHz,
                hysteresis: Hysteresis::FourC,
            };

            let result = tmp108.configure(config).await;
            assert!(result.is_ok());

            let result = tmp108.read_configuration().await;
            assert!(result.is_ok());

            let new_config = result.unwrap();
            assert_eq!(config, new_config);

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn read_temperature_default_address() {
            let expectations = vec![
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x7f, 0xf0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x64, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x4b, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x40])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xff, 0xc0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xe7, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xc9, 0x00])],
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            for (e, t) in expectations.iter().zip(temps.iter()) {
                let mock = Mock::new(e);
                let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);
                let result = tmp108.temperature().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);

                let mut mock = tmp108.destroy();
                mock.done();
            }
        }

        #[tokio::test]
        async fn set_and_read_high_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x03, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x03], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x03, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x03], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x03, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_high_limit(limit).await;
                assert!(result.is_ok());

                let result = tmp108.high_limit().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn set_and_read_low_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x02, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x02], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x02, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x02], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x02, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_low_limit(limit).await;
                assert!(result.is_ok());

                let result = tmp108.low_limit().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        /// The `set_*_limit` methods can no longer reject their input —
        /// [`Celsius`] carries the range invariant. The remaining
        /// f32 boundary is the `TemperatureThresholdSet` trait, which
        /// must still reject unrepresentable degrees before any bus
        /// traffic occurs.
        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn threshold_set_rejects_unrepresentable_degrees() {
            use embedded_sensors_hal_async::temperature::TemperatureThresholdSet;

            // No I2C transactions are expected.
            let mock = Mock::new(&[]);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            // Values outside the representable [-128.0, 127.9375] range.
            // Note 127.940 now *rounds* to 127.9375 and is accepted, so the
            // rejection cases start half an LSB beyond the endpoints.
            for bad in [128.0_f32, 127.98_f32, -128.05_f32, -200.0_f32, 200.0_f32] {
                assert!(matches!(
                    tmp108.set_temperature_threshold_low(bad).await,
                    Err(Error::InvalidInput)
                ));
                assert!(matches!(
                    tmp108.set_temperature_threshold_high(bad).await,
                    Err(Error::InvalidInput)
                ));
            }

            // Non-finite values.
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert!(matches!(
                    tmp108.set_temperature_threshold_low(bad).await,
                    Err(Error::InvalidInput)
                ));
                assert!(matches!(
                    tmp108.set_temperature_threshold_high(bad).await,
                    Err(Error::InvalidInput)
                ));
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn probe_returns_true_for_por_value() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe().await, Ok(true));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn probe_returns_false_for_non_por_value() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe().await, Ok(false));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn continuous_runs_shutdown_when_closure_returns_err() {
            // Expectations:
            //  1. Enter Continuous: read cfg, write cfg with M=Continuous.
            //  2. Closure causes one temperature read that returns a bus error.
            //  3. Cleanup must still run: read cfg, write cfg with M=Shutdown.
            //
            // If the cleanup is skipped (the pre-fix behavior) the mock will
            // panic at destroy() because the last two expectations were not
            // consumed.
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])
                    .with_error(embedded_hal::i2c::ErrorKind::Other),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            let result = tmp108
                .continuous(async |t| {
                    let _ = t.temperature().await?;
                    Ok(())
                })
                .await;

            // The closure's error must be propagated (closure error wins
            // over shutdown success).
            assert!(result.is_err());

            // All five expected transactions consumed -> the cleanup
            // shutdown ran.
            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn continuous_returns_closure_error_when_shutdown_also_fails() {
            use embedded_hal_async::i2c::Error as _;

            // Closure fails AND shutdown fails. The closure's error must win.
            let closure_err = embedded_hal::i2c::ErrorKind::Bus;
            let shutdown_err = embedded_hal::i2c::ErrorKind::ArbitrationLoss;

            let expectations = vec![
                // Enter Continuous.
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
                // Closure errors.
                Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]).with_error(closure_err),
                // Shutdown read errors too.
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]).with_error(shutdown_err),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            let result = tmp108
                .continuous(async |t| {
                    let _ = t.temperature().await?;
                    Ok(())
                })
                .await;

            // Must propagate the closure error, not the shutdown error.
            assert_eq!(result.err().map(|e| e.kind()), Some(closure_err));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn handle_threshold_alerts_properly() {
            use embedded_hal_mock::eh1::digital;
            use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};

            // Sensor i2c bus mocks and expectations
            let i2c_expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
                Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
                Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00]),
            ];
            let i2c_mock = Mock::new(&i2c_expectations);

            // Threshold alert GPIO pin mocks and expectations
            let pin_expectations = vec![digital::Transaction::wait_for_edge(digital::Edge::Falling)];
            let pin_mock = digital::Mock::new(&pin_expectations);

            // Create a ALERTTMP108 instance and configure it as active-low interrupt mode
            let mut tmp108 = AlertTmp108::new_with_a0_gnd(i2c_mock, pin_mock);

            let cfg = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveLow,
                ..Default::default()
            };

            let result = tmp108.sensor_mut().configure(cfg).await;
            assert!(result.is_ok());

            // Set alert thresholds
            let result = tmp108.set_temperature_threshold_low(25.0).await;
            assert!(result.is_ok());
            let result = tmp108.set_temperature_threshold_high(80.0).await;
            assert!(result.is_ok());

            // Ensure alert pin waits for a falling edge
            let result = tmp108.wait_for_temperature_threshold().await;
            assert!(result.is_ok());

            // Check that recently sampled temperature is returned
            let temp = result.unwrap();
            assert_approx_eq!(temp, 80.0, 1e-4);

            let (mut i2c_mock, mut pin_mock) = tmp108.destroy();
            i2c_mock.done();
            pin_mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn hysteresis_snaps_within_tolerance() {
            use embedded_sensors_hal_async::temperature::TemperatureHysteresis;

            // For each legal value, every input within 0.05 °C must
            // succeed and program the corresponding chip setting (no I2C
            // mismatch). Each acceptance path performs: read cfg, write cfg.
            //
            // - 0.0 °C snaps to Hysteresis::ZeroC => HYS bits 0b00 => cfg word 0x0022 -> bytes [0x22, 0x00]
            // - 1.0 °C snaps to Hysteresis::OneC => HYS bits 0b01 => cfg word 0x1022 -> bytes [0x22, 0x10]
            // - 2.0 °C snaps to Hysteresis::TwoC => HYS bits 0b10 => cfg word 0x2022 -> bytes [0x22, 0x20]
            // - 4.0 °C snaps to Hysteresis::FourC => HYS bits 0b11 => cfg word 0x3022 -> bytes [0x22, 0x30]
            //
            // For each accepted input we expect: write-read of cfg, then a
            // write of the new cfg. The chip's POR is 0x1022 (HYS=01).
            let cases: &[(f32, [u8; 2])] = &[
                // Exact-match accepted values.
                (0.0, [0x22, 0x00]),
                (1.0, [0x22, 0x10]),
                (2.0, [0x22, 0x20]),
                (4.0, [0x22, 0x30]),
                // Within-tolerance inputs that previously failed under
                // f32::EPSILON snapping.
                (0.04_f32, [0x22, 0x00]),
                (1.000_000_1_f32, [0x22, 0x10]),
                (0.1_f32 + 0.9_f32, [0x22, 0x10]),
                (1.95_f32, [0x22, 0x20]),
                (3.97_f32, [0x22, 0x30]),
            ];

            // Each accepted input triggers:
            //   1. read_configuration() in the hysteresis impl: write_read
            //   2. configure() -> modify (read-modify-write): write_read + write
            // The current cfg byte stream is the chip's POR value 0x1022 ->
            // [0x22, 0x10]. After snapping the result is reflected in the
            // HYS bits of the final write.
            let mut expectations = Vec::new();
            for (_, written) in cases {
                // read_configuration
                expectations.push(Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]));
                // configure -> modify: read
                expectations.push(Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]));
                // configure -> modify: write
                expectations.push(Transaction::write(0x48, vec![0x01, written[0], written[1]]));
            }

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for (input, _) in cases {
                let r = tmp108.set_temperature_threshold_hysteresis(*input).await;
                assert!(r.is_ok(), "input {input} should be accepted");
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn hysteresis_rejects_out_of_tolerance_and_non_finite() {
            use embedded_sensors_hal_async::temperature::TemperatureHysteresis;

            // No I2C transactions expected; out-of-tolerance and non-finite
            // inputs must be rejected before any bus traffic.
            let mock = Mock::new(&[]);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for bad in [-0.5_f32, 0.5_f32, 3.0_f32, 5.0_f32, -1.0_f32, 10.0_f32] {
                let r = tmp108.set_temperature_threshold_hysteresis(bad).await;
                assert!(
                    matches!(r, Err(Error::InvalidInput)),
                    "input {bad} should be rejected as InvalidInput, got {r:?}"
                );
            }

            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                let r = tmp108.set_temperature_threshold_hysteresis(bad).await;
                assert!(
                    matches!(r, Err(Error::InvalidInput)),
                    "input {bad} should be rejected as InvalidInput, got {r:?}"
                );
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn alert_pin_error_is_propagated_as_error_pin() {
            use embedded_hal_mock::eh1::{MockError, digital};
            use embedded_sensors_hal_async::temperature::TemperatureThresholdWait;

            // Configure for Interrupt + ActiveLow so wait_for_falling_edge
            // is the gating operation. The pin then errors; the driver
            // must surface the GPIO error via Error::Pin(_), not swallow
            // it (the pre-fix behavior collapsed all GPIO failures to
            // Error::Other).
            let i2c_expectations = vec![
                // configure: read + write
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
                // wait_for_temperature_threshold reads cfg first
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
            ];
            let i2c_mock = Mock::new(&i2c_expectations);

            let pin_err = MockError::Io(std::io::ErrorKind::Other);
            let pin_expectations =
                vec![digital::Transaction::wait_for_edge(digital::Edge::Falling).with_error(pin_err.clone())];
            let pin_mock = digital::Mock::new(&pin_expectations);

            let mut tmp108 = AlertTmp108::new_with_a0_gnd(i2c_mock, pin_mock);

            let cfg = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveLow,
                ..Default::default()
            };
            tmp108.sensor_mut().configure(cfg).await.unwrap();

            let result = tmp108.wait_for_temperature_threshold().await;
            match result {
                Err(Error::Pin(e)) => assert_eq!(e, pin_err),
                other => panic!("expected Error::Pin, got {other:?}"),
            }

            let (mut i2c_mock, mut pin_mock) = tmp108.destroy();
            i2c_mock.done();
            pin_mock.done();
        }
    }
}
