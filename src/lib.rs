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
//! methods (notably [`Tmp108::configure`], [`Tmp108::one_shot`], and
//! [`Tmp108::shutdown`]) perform a read-modify-write on the
//! configuration register as two distinct I²C transactions. On a
//! multi-master bus, a second master writing to register `0x01`
//! between the read and the write will silently lose those writes —
//! the TMP108 has no register-level lock. In multi-master designs,
//! serialise driver access at a higher level (e.g. a bus mutex around
//! the entire driver, not just individual I²C transactions).
//!
//! ## Driver lifecycle on drop
//!
//! Dropping a [`Tmp108`] or [`AlertTmp108`] does **not** change the
//! chip's operating mode. The chip retains whatever `M` bits were last
//! written. If you want the chip to stop drawing current after the
//! driver goes out of scope, call [`Tmp108::shutdown`] (or finish a
//! [`Tmp108::continuous`] call cleanly) before dropping.
//!
//! In particular, dropping the future returned by
//! [`Tmp108::continuous`] mid-flight (e.g. via `embassy_futures::select!`
//! or `tokio::time::timeout`) leaves the chip in `Mode::Continuous`
//! indefinitely. See the cancel-safety note on that method.

#![doc(html_root_url = "https://docs.rs/tmp108/latest")]
#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "async")]
use device_driver::AsyncRegisterInterface;
#[cfg(not(feature = "async"))]
use device_driver::RegisterInterface;
#[cfg(not(feature = "async"))]
use embedded_hal::{delay::DelayNs, i2c::I2c};
#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayNs as AsyncDelayNs, i2c::I2c as AsyncI2c};

#[allow(clippy::all)]
#[allow(clippy::pedantic)]
#[allow(missing_docs)]
#[allow(unsafe_code)]
#[allow(unused)]
mod inner;

use crate::inner::Inner;
use crate::inner::field_sets::{THigh, TLow};
pub use crate::inner::{ConversionRate, Hysteresis, Mode, Polarity, Thermostat};

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
#[derive(Clone, Copy, Debug, PartialEq)]
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
            conversion_rate: ConversionRate::_1Hz,
            hysteresis: Hysteresis::_1C,
        }
    }
}

/// Tmp108 device driver.
#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Tmp108", idents(AsyncI2c(sync = "I2c"))),
    async(feature = "async", keep_self)
)]
pub struct Tmp108<I2C: AsyncI2c> {
    inner: Inner<Interface<I2C>>,
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Tmp108", idents(AsyncI2c(sync = "I2c"))),
    async(feature = "async", keep_self)
)]
impl<I2C: AsyncI2c> Tmp108<I2C> {
    /// Create a new TMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::{A0, Tmp108};
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new(i2c, A0::Sda);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = Interface::new(i2c, a0);
        let inner = Inner::new(interface);

        Self { inner }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
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
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_vplus(i2c);
    /// assert_eq!(tmp.addr(), 0x49);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
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
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_sda(i2c);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
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
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_scl(i2c);
    /// assert_eq!(tmp.addr(), 0x4b);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
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
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn addr(&self) -> u8 {
        self.inner.interface.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut i2c = tmp.destroy();
    /// i2c.done();
    /// # }
    /// ```
    pub fn destroy(self) -> I2C {
        self.inner.interface.i2c
    }

    /// Create a new `AlertTmp108` instance by consuming the original Tmp108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let alert_tmp = tmp.into_alert(alert);
    /// let (mut i2c, mut alert) = alert_tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # }
    /// ```
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
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
    /// Underlying TMP108 sensor.
    pub tmp108: Tmp108<I2C>,
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
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::{A0, AlertTmp108};
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new(i2c, A0::Sda, alert);
    /// assert_eq!(tmp.tmp108.addr(), 0x4a);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # }
    /// ```
    pub fn new(i2c: I2C, a0: A0, alert: ALERT) -> Self {
        let tmp108 = Tmp108::new(i2c, a0);
        Self { tmp108, alert }
    }

    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// assert_eq!(tmp.tmp108.addr(), 0x48);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # }
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
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_vplus(i2c, alert);
    /// assert_eq!(tmp.tmp108.addr(), 0x49);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # }
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
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_sda(i2c, alert);
    /// assert_eq!(tmp.tmp108.addr(), 0x4a);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # }
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
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_scl(i2c, alert);
    /// assert_eq!(tmp.tmp108.addr(), 0x4b);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # }
    /// ```
    pub fn new_with_a0_scl(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Scl, alert)
    }

    /// Destroy the driver instance, return the I2C bus instance and ALERT pin instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let (mut i2c, mut alert) = tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # }
    /// ```
    pub fn destroy(self) -> (I2C, ALERT) {
        (self.tmp108.destroy(), self.alert)
    }
}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "Tmp108",
        idents(AsyncI2c(sync = "I2c"), AsyncDelayNs(sync = "DelayNs"))
    ),
    async(feature = "async", keep_self)
)]
impl<I2C: AsyncI2c> Tmp108<I2C> {
    const CELSIUS_PER_BIT: f32 = 0.0625;

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
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// // Chip returns the POR configuration -> probe() reports true.
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert!(tmp.probe().unwrap());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn probe(&mut self) -> Result<bool, I2C::Error> {
        /// TMP108 configuration register reset value, per the datasheet.
        const POR: u16 = 0x1022;

        #[cfg(feature = "async")]
        let raw = self.inner.configuration().read_async().await?;

        #[cfg(not(feature = "async"))]
        let raw = self.inner.configuration().read()?;

        Ok(u16::from_le_bytes(raw.into()) == POR)
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::{Config, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let cfg = tmp.read_configuration().unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        #[cfg(feature = "async")]
        let c = self.inner.configuration().read_async().await?;

        #[cfg(not(feature = "async"))]
        let c = self.inner.configuration().read()?;

        Ok(Config {
            thermostat_mode: c.tm(),
            alert_polarity: c.pol(),
            conversion_rate: c.cr(),
            hysteresis: c.hys(),
        })
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.configure(Config {
    ///     thermostat_mode: Thermostat::Interrupt,
    ///     alert_polarity: Polarity::ActiveHigh,
    ///     conversion_rate: ConversionRate::_16Hz,
    ///     hysteresis: Hysteresis::_4C,
    /// }).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        #[cfg(feature = "async")]
        let res = self
            .inner
            .configuration()
            .modify_async(|r| {
                r.set_tm(config.thermostat_mode);
                r.set_pol(config.alert_polarity);
                r.set_cr(config.conversion_rate);
                r.set_hys(config.hysteresis);
            })
            .await;

        #[cfg(not(feature = "async"))]
        let res = self.inner.configuration().modify(|r| {
            r.set_tm(config.thermostat_mode);
            r.set_pol(config.alert_polarity);
            r.set_cr(config.conversion_rate);
            r.set_hys(config.hysteresis);
        });

        res
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let temp = tmp.temperature().unwrap();
    /// assert!((temp - 50.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
        #[cfg(feature = "async")]
        let res = self.inner.temperature().read_async().await;

        #[cfg(not(feature = "async"))]
        let res = self.inner.temperature().read();

        let raw = res?;
        Ok(Self::to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x21, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.one_shot().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn one_shot(&mut self) -> Result<(), I2C::Error> {
        #[cfg(feature = "async")]
        let res = self
            .inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::OneShot))
            .await;

        #[cfg(not(feature = "async"))]
        let res = self.inner.configuration().modify(|r| r.set_m(Mode::OneShot));

        res
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.shutdown().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        #[cfg(feature = "async")]
        let res = self
            .inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Shutdown))
            .await;

        #[cfg(not(feature = "async"))]
        let res = self.inner.configuration().modify(|r| r.set_m(Mode::Shutdown));

        res
    }

    #[cfg(feature = "async")]
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
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// use tmp108::Tmp108;
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
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
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
    /// Reads the configuration register to discover the current
    /// [`ConversionRate`], delays for one period (1/CR), and then reads
    /// the temperature register. Intended for callers driving the chip
    /// in [`Mode::Continuous`] (typically from inside a
    /// [`continuous`][Self::continuous] closure) who want to align
    /// reads with the chip's conversion cadence.
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
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the calls.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::delay::NoopDelay;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut delay = NoopDelay::new();
    /// let temp = tmp.wait_for_temperature(&mut delay).unwrap();
    /// assert!((temp - 50.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn wait_for_temperature<DELAY: AsyncDelayNs>(&mut self, delay: &mut DELAY) -> Result<f32, I2C::Error> {
        let config = self.read_configuration().await?;

        let delay_time_us = match config.conversion_rate {
            ConversionRate::_0_25Hz => 4_000_000,
            ConversionRate::_1Hz => 1_000_000,
            ConversionRate::_4Hz => 250_000,
            ConversionRate::_16Hz => 62_500,
        };

        delay.delay_us(delay_time_us).await;
        self.temperature().await
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.low_limit().unwrap();
    /// assert!((limit - 25.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        #[cfg(feature = "async")]
        let raw = self.inner.t_low().read_async().await?;

        #[cfg(not(feature = "async"))]
        let raw = self.inner.t_low().read()?;
        Ok(Self::to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// - `Error::InvalidInput` if `limit` is NaN, ±∞, or outside
    ///   `[-128.0, 127.9375] °C` (the representable range of the chip's
    ///   12-bit fixed-point limit register).
    /// - `Error::Bus` when the I2C transaction fails.
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_low_limit(25.0).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), Error<I2C::Error>> {
        let raw = Self::to_raw(limit).ok_or(Error::InvalidInput)?.to_be_bytes();

        #[cfg(feature = "async")]
        self.inner
            .t_low()
            .write_async(|r| *r = TLow::from(raw))
            .await
            .map_err(Error::Bus)?;

        #[cfg(not(feature = "async"))]
        self.inner.t_low().write(|r| *r = TLow::from(raw)).map_err(Error::Bus)?;

        Ok(())
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.high_limit().unwrap();
    /// assert!((limit - 80.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        #[cfg(feature = "async")]
        let raw = self.inner.t_high().read_async().await?;

        #[cfg(not(feature = "async"))]
        let raw = self.inner.t_high().read()?;
        Ok(Self::to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// - `Error::InvalidInput` if `limit` is NaN, ±∞, or outside
    ///   `[-128.0, 127.9375] °C` (the representable range of the chip's
    ///   12-bit fixed-point limit register).
    /// - `Error::Bus` when the I2C transaction fails.
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # #[cfg(feature = "async")] fn main() {}
    /// # #[cfg(not(feature = "async"))]
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_high_limit(80.0).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), Error<I2C::Error>> {
        let raw = Self::to_raw(limit).ok_or(Error::InvalidInput)?.to_be_bytes();

        #[cfg(feature = "async")]
        self.inner
            .t_high()
            .write_async(|r| *r = THigh::from(raw))
            .await
            .map_err(Error::Bus)?;

        #[cfg(not(feature = "async"))]
        self.inner
            .t_high()
            .write(|r| *r = THigh::from(raw))
            .map_err(Error::Bus)?;

        Ok(())
    }

    fn to_celsius(t: i16) -> f32 {
        // Per the datasheet the temperature and limit registers are
        // left-aligned 12-bit signed values: the LSB is the bit at
        // position 4 (0.0625 °C/LSB) and bits 3..0 are reserved=0.
        // Compute the conversion as a single multiplication by
        // CELSIUS_PER_BIT/16 (== 1/256 = 0.003_906_25, exactly
        // representable in f32) instead of dividing by 16 first via
        // integer division.
        //
        // For datasheet-conforming inputs (bits 3..0 == 0) this returns
        // exactly the same f32 as before. For any non-zero low bits it
        // returns the correct value instead of truncating toward zero
        // (which was asymmetric for negative inputs).
        f32::from(t) * (Self::CELSIUS_PER_BIT / 16.0)
    }

    /// Convert a temperature in degrees Celsius to the raw 12-bit signed
    /// fixed-point representation used by the chip's TLow/THigh registers.
    ///
    /// Returns `None` for inputs that are NaN, ±∞, or outside the
    /// representable range `[-128.0, 127.9375] °C`.
    fn to_raw(t: f32) -> Option<i16> {
        // i16 representable range divided by the 16x scale factor.
        const MIN: f32 = -128.0;
        const MAX: f32 = 127.937_5;

        if !t.is_finite() || !(MIN..=MAX).contains(&t) {
            return None;
        }

        // Range-checked above; this cast cannot truncate.
        #[allow(clippy::cast_possible_truncation)]
        Some((t * 16.0 / Self::CELSIUS_PER_BIT) as i16)
    }
}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "Interface",
        idents(AsyncI2c(sync = "I2c"), AsyncRegisterInterface(sync = "RegisterInterface"))
    ),
    async(feature = "async", keep_self)
)]
struct Interface<I2C: AsyncI2c> {
    i2c: I2C,
    addr: u8,
}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "Interface",
        idents(AsyncI2c(sync = "I2c"), AsyncRegisterInterface(sync = "RegisterInterface"))
    ),
    async(feature = "async", keep_self)
)]
impl<I2C: AsyncI2c> Interface<I2C> {
    /// Create a new Interface instance.
    fn new(i2c: I2C, a0: A0) -> Self {
        Self { i2c, addr: a0.into() }
    }
}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "Interface",
        idents(AsyncI2c(sync = "I2c"), AsyncRegisterInterface(sync = "RegisterInterface"))
    ),
    async(feature = "async", keep_self)
)]
impl<I2C: AsyncI2c> AsyncRegisterInterface for Interface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0; 3];

        buf[0] = address;
        buf[1..].copy_from_slice(data);

        self.i2c.write(self.addr, &buf).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
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
#[derive(Debug)]
pub enum Error<E: embedded_hal::i2c::Error, P: embedded_hal::digital::Error = core::convert::Infallible> {
    /// I2C bus error.
    Bus(E),
    /// Input failed validation (out of range, NaN, ±∞, unsupported value).
    InvalidInput,
    /// ALERT pin GPIO error.
    Pin(P),
}

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
        self.temperature().map_err(Error::Bus)
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
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::sensor::ErrorType for Tmp108<I2C> {
    type Error = Error<I2C::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureSensor for Tmp108<I2C> {
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.temperature().await.map_err(Error::Bus)
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
        self.tmp108.temperature().await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureThresholdSet
    for Tmp108<I2C>
{
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.set_low_limit(threshold).await
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.set_high_limit(threshold).await
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
        // Bare-Tmp108 set_*_limit returns Error<I2C::Error>; widen the Pin
        // type parameter to the AlertTmp108-flavored Error.
        self.tmp108.set_low_limit(threshold).await.map_err(widen_pin_err)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108.set_high_limit(threshold).await.map_err(widen_pin_err)
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
        Ok(temperature)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureHysteresis for Tmp108<I2C> {
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        // The trait method takes a continuous range of f32 °C values, but
        // the chip only supports four discrete hysteresis settings:
        // 0, 1, 2, and 4 °C. Snap the input to the nearest legal value
        // within a tolerance band of 0.05 °C; reject anything outside the
        // band (and any non-finite input) with `Error::InvalidInput`.
        //
        // The tolerance is generous enough to absorb ordinary float
        // arithmetic (e.g. 0.1 + 0.9 rounding away from 1.0) while still
        // surfacing genuinely unsupported requests like 3.0 °C.
        const HYS_VALUES: &[(f32, Hysteresis)] = &[
            (0.0, Hysteresis::_0C),
            (1.0, Hysteresis::_1C),
            (2.0, Hysteresis::_2C),
            (4.0, Hysteresis::_4C),
        ];
        const HYS_TOLERANCE: f32 = 0.05;

        if !hysteresis.is_finite() {
            return Err(Error::InvalidInput);
        }

        // HYS_VALUES is non-empty so min_by always returns Some.
        let (closest, snapped) = HYS_VALUES
            .iter()
            .copied()
            .min_by(|(a, _), (b, _)| (hysteresis - a).abs().total_cmp(&(hysteresis - b).abs()))
            .expect("HYS_VALUES is non-empty");

        if (hysteresis - closest).abs() > HYS_TOLERANCE {
            return Err(Error::InvalidInput);
        }

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
/// `Tmp108` methods (which cannot themselves produce a `Pin` error).
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
    use super::inner::field_sets::Configuration;
    use super::*;

    #[test]
    fn default_configuration() {
        let cfg = Configuration::new();
        assert_eq!(u16::from_le_bytes(cfg.into()), 0x1022);
    }

    #[test]
    fn modify_mode() {
        let mut cfg = Configuration::new();
        cfg.set_m(Mode::Shutdown);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1020);
        cfg.set_m(Mode::OneShot);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1021);
        cfg.set_m(Mode::Continuous);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
    }

    #[test]
    fn modify_thermostat_mode() {
        let mut cfg = Configuration::new();
        cfg.set_tm(Thermostat::Comparator);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_tm(Thermostat::Interrupt);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1026);
    }

    #[test]
    fn modify_watchdog_temperature_flags() {
        let mut cfg = Configuration::new();
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
        let mut cfg = Configuration::new();
        cfg.set_cr(ConversionRate::_0_25Hz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1002);
        cfg.set_cr(ConversionRate::_1Hz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_cr(ConversionRate::_4Hz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1042);
        cfg.set_cr(ConversionRate::_16Hz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1062);
    }

    #[test]
    fn modify_hysteresis() {
        let mut cfg = Configuration::new();
        cfg.set_hys(Hysteresis::_0C);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x0022);
        cfg.set_hys(Hysteresis::_1C);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_hys(Hysteresis::_2C);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x2022);
        cfg.set_hys(Hysteresis::_4C);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x3022);
    }

    #[test]
    fn modify_polarity() {
        let mut cfg = Configuration::new();
        cfg.set_pol(Polarity::ActiveLow);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_pol(Polarity::ActiveHigh);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x9022);
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
                conversion_rate: ConversionRate::_16Hz,
                hysteresis: Hysteresis::_4C,
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
                assert_approx_eq!(temp, *t, 1e-4);

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
                let result = tmp108.set_low_limit(*t);
                assert!(result.is_ok());

                let result = tmp108.low_limit();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp, *t, 1e-4);
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
                let result = tmp108.set_high_limit(*t);
                assert!(result.is_ok());

                let result = tmp108.high_limit();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp, *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn reject_invalid_set_limit_inputs() {
            // No I2C transactions are expected; out-of-range / non-finite
            // inputs must be rejected before any bus traffic.
            let mock = Mock::new(&[]);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            // Values just outside the representable [-128.0, 127.9375] range.
            for bad in [128.0_f32, 127.940_f32, -128.001_f32, -200.0_f32, 200.0_f32] {
                assert!(matches!(tmp108.set_low_limit(bad), Err(Error::InvalidInput)));
                assert!(matches!(tmp108.set_high_limit(bad), Err(Error::InvalidInput)));
            }

            // Non-finite values.
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert!(matches!(tmp108.set_low_limit(bad), Err(Error::InvalidInput)));
                assert!(matches!(tmp108.set_high_limit(bad), Err(Error::InvalidInput)));
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn to_celsius_is_symmetric_around_zero() {
            // For datasheet-conforming inputs (bits 3..0 == 0) the
            // conversion is identical to the previous integer-division
            // implementation. For inputs with non-zero low bits the new
            // implementation no longer truncates toward zero, which was
            // asymmetric for negative values.
            //
            // 0xFFFF as i16 = -1. Old code: f32::from(-1 / 16) * 0.0625 =
            // f32::from(0) * 0.0625 = 0.0. New code: f32::from(-1) *
            // (0.0625 / 16.0) = -0.003_906_25.
            let cases: &[(i16, f32)] = &[
                (0x0000, 0.0),
                (0x0010, 0.0625),
                (0xFFF0_u16 as i16, -0.0625),
                (0x7FF0, 127.9375),
                (0xC900_u16 as i16, -55.0),
                // Non-zero low bits: symmetric resolution.
                (0x0001, 0.003_906_25),
                (0xFFFF_u16 as i16, -0.003_906_25),
            ];
            for (raw, expected) in cases {
                let got = Tmp108::<Mock>::to_celsius(*raw);
                assert_approx_eq!(got, *expected, 1e-5);
            }
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

        #[tokio::test]
        async fn change_configuration() {
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
                Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
            ];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
            let result = tmp108.read_configuration().await;
            assert!(result.is_ok());

            let config = result.unwrap();
            assert_eq!(config, Config::default());

            let config = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::_16Hz,
                hysteresis: Hysteresis::_4C,
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
                let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
                let result = tmp108.temperature().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp, *t, 1e-4);

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
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let result = tmp108.set_high_limit(*t).await;
                assert!(result.is_ok());

                let result = tmp108.high_limit().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp, *t, 1e-4);
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
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let result = tmp108.set_low_limit(*t).await;
                assert!(result.is_ok());

                let result = tmp108.low_limit().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp, *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn reject_invalid_set_limit_inputs() {
            // No I2C transactions are expected; out-of-range / non-finite
            // inputs must be rejected before any bus traffic.
            let mock = Mock::new(&[]);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            // Values just outside the representable [-128.0, 127.9375] range.
            for bad in [128.0_f32, 127.940_f32, -128.001_f32, -200.0_f32, 200.0_f32] {
                assert!(matches!(tmp108.set_low_limit(bad).await, Err(Error::InvalidInput)));
                assert!(matches!(tmp108.set_high_limit(bad).await, Err(Error::InvalidInput)));
            }

            // Non-finite values.
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert!(matches!(tmp108.set_low_limit(bad).await, Err(Error::InvalidInput)));
                assert!(matches!(tmp108.set_high_limit(bad).await, Err(Error::InvalidInput)));
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn probe_returns_true_for_por_value() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe().await, Ok(true));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn probe_returns_false_for_non_por_value() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

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
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

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
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

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

            let result = tmp108.tmp108.configure(cfg).await;
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
            // - 0.0 °C snaps to Hysteresis::_0C => HYS bits 0b00 => cfg word 0x0022 -> bytes [0x22, 0x00]
            // - 1.0 °C snaps to Hysteresis::_1C => HYS bits 0b01 => cfg word 0x1022 -> bytes [0x22, 0x10]
            // - 2.0 °C snaps to Hysteresis::_2C => HYS bits 0b10 => cfg word 0x2022 -> bytes [0x22, 0x20]
            // - 4.0 °C snaps to Hysteresis::_4C => HYS bits 0b11 => cfg word 0x3022 -> bytes [0x22, 0x30]
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
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

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
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

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
            tmp108.tmp108.configure(cfg).await.unwrap();

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
