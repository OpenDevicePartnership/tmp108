//! This is a platform-agnostic Rust driver for the TMP108 temperature sensor
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.ti.com/lit/gpn/tmp108

#![doc(html_root_url = "https://docs.rs/tmp108/latest")]
#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "async")]
use core::future::Future;

#[cfg(feature = "async")]
use device_driver::AsyncRegisterInterface;
#[cfg(not(feature = "async"))]
use device_driver::RegisterInterface;
#[cfg(not(feature = "async"))]
use embedded_hal::{delay::DelayNs, i2c::I2c};
#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayNs as AsyncDelayNs, i2c::I2c as AsyncI2c};

// inner is auto-generated, we don't want build to fail because of auto-generated code.
#[allow(unused, unsafe_code, clippy::all)]
mod inner;

use crate::inner::field_sets::{THigh, TLow};
use crate::inner::{ConversionRate, Hysteresis, Inner, Mode, Polarity, Thermostat};

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
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = Interface::new(i2c, a0);
        let inner = Inner::new(interface);

        Self { inner }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    pub fn addr(&self) -> u8 {
        self.inner.interface.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.inner.interface.i2c
    }

    /// Create a new `AlertTmp108` instance by consuming the original Tmp108 instance.
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, ALERT> {
        AlertTmp108 { tmp108: self, alert }
    }
}

/// Tmp108 asynchronous device driver (with alert pin)
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
    pub fn new(i2c: I2C, a0: A0, alert: ALERT) -> Self {
        let tmp108 = Tmp108::new(i2c, a0);
        Self { tmp108, alert }
    }

    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Gnd, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to V+, resulting in an
    /// instance responding to address `0x49`.
    pub fn new_with_a0_vplus(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Vplus, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SDA, resulting in an
    /// instance responding to address `0x4a`.
    pub fn new_with_a0_sda(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Sda, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SCL, resulting in an
    /// instance responding to address `0x4b`.
    pub fn new_with_a0_scl(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Scl, alert)
    }

    /// Destroy the driver instance, return the I2C bus instance and ALERT pin instance.
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

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
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
    pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
        #[cfg(feature = "async")]
        let res = self.inner.temperature().read_async().await;

        #[cfg(not(feature = "async"))]
        let res = self.inner.temperature().read();

        let raw = res?;
        Ok(Self::to_celsius(i16::from_ne_bytes(raw.into())))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
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
    /// Initiate continuous conversions
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn continuous<F, Fut>(&mut self, f: F) -> Result<(), I2C::Error>
    where
        F: FnOnce(&mut Self) -> Fut,
        Fut: Future<Output = Result<(), I2C::Error>> + Send,
    {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Continuous))
            .await?;

        f(self).await?;
        self.shutdown().await
    }

    /// Wait for conversion to complete. This method will block for the amount
    /// of time dictated by the CR bits in the [`Configuration`]
    /// register. Caller is required to call this method from within their
    /// continuous conversion closure.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
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
    pub async fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        #[cfg(feature = "async")]
        let raw = self.inner.t_low().read_async().await?;

        #[cfg(not(feature = "async"))]
        let raw = self.inner.t_low().read()?;
        Ok(Self::to_celsius(i16::from_le_bytes(raw.into())))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = Self::to_raw(limit).to_le_bytes();

        #[cfg(feature = "async")]
        self.inner.t_low().write_async(|r| *r = TLow::from(raw)).await?;

        #[cfg(not(feature = "async"))]
        self.inner.t_low().write(|r| *r = TLow::from(raw))?;

        Ok(())
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        #[cfg(feature = "async")]
        let raw = self.inner.t_high().read_async().await?;

        #[cfg(not(feature = "async"))]
        let raw = self.inner.t_high().read()?;
        Ok(Self::to_celsius(i16::from_le_bytes(raw.into())))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = Self::to_raw(limit).to_le_bytes();

        #[cfg(feature = "async")]
        self.inner.t_high().write_async(|r| *r = THigh::from(raw)).await?;

        #[cfg(not(feature = "async"))]
        self.inner.t_high().write(|r| *r = THigh::from(raw))?;

        Ok(())
    }

    fn to_celsius(t: i16) -> f32 {
        f32::from(t / 16) * Self::CELSIUS_PER_BIT
    }

    #[allow(clippy::cast_possible_truncation)]
    fn to_raw(t: f32) -> i16 {
        (t * 16.0 / Self::CELSIUS_PER_BIT) as i16
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
#[derive(Debug)]
pub enum Error<E: embedded_hal::i2c::Error> {
    /// I2C Bus Error
    Bus(E),
    /// Invalid Input Error
    InvalidInput,
    /// Other Error
    Other,
}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<E: embedded_hal::i2c::Error> embedded_sensors_hal::sensor::Error for Error<E> {
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
impl<E: embedded_hal_async::i2c::Error> embedded_sensors_hal_async::sensor::Error for Error<E> {
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
    type Error = Error<I2C::Error>;
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
        self.set_low_limit(threshold).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.set_high_limit(threshold).await.map_err(Error::Bus)
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
        self.tmp108.set_low_limit(threshold).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108.set_high_limit(threshold).await.map_err(Error::Bus)
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
                self.alert.wait_for_low().await.map_err(|_| Error::Other)?;
            }
            (Thermostat::Comparator, Polarity::ActiveHigh) => {
                self.alert.wait_for_high().await.map_err(|_| Error::Other)?;
            }

            // In interrupt mode, the ALERT pin is immediately reset (by reading config register)
            // after triggering.
            //
            // If called in a loop, next iteration would wait even if temperature remains outside
            // threshold.
            (Thermostat::Interrupt, Polarity::ActiveLow) => {
                self.alert.wait_for_falling_edge().await.map_err(|_| Error::Other)?;
                let _ = self.tmp108.read_configuration().await.map_err(Error::Bus)?;
            }
            (Thermostat::Interrupt, Polarity::ActiveHigh) => {
                self.alert.wait_for_rising_edge().await.map_err(|_| Error::Other)?;
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
        // Trait method takes a continuous range of f32 values as argument, but internally driver
        // only accepts 4 discrete values for hysteresis.
        //
        // We ensure only a correct value for hysteresis is passed in, and return error otherwise.
        let hysteresis = if (hysteresis - 0.0).abs() < f32::EPSILON {
            Hysteresis::_0C
        } else if (hysteresis - 1.0).abs() < f32::EPSILON {
            Hysteresis::_1C
        } else if (hysteresis - 2.0).abs() < f32::EPSILON {
            Hysteresis::_2C
        } else if (hysteresis - 4.0).abs() < f32::EPSILON {
            Hysteresis::_4C
        } else {
            return Err(Error::InvalidInput);
        };

        let mut config = self.read_configuration().await.map_err(|_| Error::Other)?;
        config.hysteresis = hysteresis;
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
        self.tmp108.set_temperature_threshold_hysteresis(hysteresis).await
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
        fn read_temperature_default_address() {
            let expectations = vec![
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xf0, 0x7f])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x64])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x50])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x4b])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x32])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x19])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x40, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xc0, 0xff])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0xe7])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0xc9])],
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
        async fn read_temperature_default_address() {
            let expectations = vec![
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xf0, 0x7f])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x64])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x50])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x4b])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x32])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x19])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x40, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xc0, 0xff])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0xe7])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0xc9])],
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

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn handle_threshold_alerts_properly() {
            use embedded_hal_mock::eh1::digital;
            use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};

            // Sensor i2c bus mocks and expectations
            let i2c_expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x19]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x50]),
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
                Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x50]),
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
    }
}
