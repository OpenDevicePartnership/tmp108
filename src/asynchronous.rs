//! Tmp108 Async API
use core::future::Future;

#[cfg(feature = "embedded-sensors-hal-async")]
use embedded_sensors_hal_async::sensor;
#[cfg(feature = "embedded-sensors-hal-async")]
use embedded_sensors_hal_async::temperature::{
    DegreesCelsius, TemperatureHysteresis, TemperatureSensor, TemperatureThresholdSet, TemperatureThresholdWait,
};

use super::{Configuration, ConversionMode, ConversionRate, Hysteresis, Register, A0};
#[cfg(feature = "embedded-sensors-hal-async")]
use super::{Polarity, ThermostatMode};

/// TMP108 asynchronous device driver
pub struct Tmp108<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> {
    /// The concrete I2C bus implementation
    i2c: I2C,

    /// The concrete [`embedded_hal::delay::DelayNs`] implementation
    delay: DELAY,

    /// The I2C address.
    pub(crate) addr: u8,

    /// A local cache of the sensor configuration.
    config: Configuration,
}

impl<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> Tmp108<I2C, DELAY> {
    const CELSIUS_PER_BIT: f32 = 0.0625;
    const CONVERSION_TIME_TYPICAL_MS: u32 = 27;

    /// Create a new TMP108 instance.
    pub async fn new_async(i2c: I2C, mut delay: DELAY, a0: A0) -> Self {
        delay.delay_ms(Self::CONVERSION_TIME_TYPICAL_MS).await;

        Self {
            i2c,
            delay,
            addr: a0.into(),
            config: Configuration::default(),
        }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    pub async fn new_async_with_a0_gnd(i2c: I2C, delay: DELAY) -> Self {
        Self::new_async(i2c, delay, A0::Gnd).await
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in an
    /// instance responding to address `0x49`.
    pub async fn new_async_with_a0_vplus(i2c: I2C, delay: DELAY) -> Self {
        Self::new_async(i2c, delay, A0::Vplus).await
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in an
    /// instance responding to address `0x4a`.
    pub async fn new_async_with_a0_sda(i2c: I2C, delay: DELAY) -> Self {
        Self::new_async(i2c, delay, A0::Sda).await
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in an
    /// instance responding to address `0x4b`.
    pub async fn new_async_with_a0_scl(i2c: I2C, delay: DELAY) -> Self {
        Self::new_async(i2c, delay, A0::Scl).await
    }

    /// Create a new ALERTTMP108 instance by consuming the original TMP108 instance.
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, DELAY, ALERT> {
        AlertTmp108 { tmp108: self, alert }
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn configuration(&mut self) -> Result<Configuration, I2C::Error> {
        let data = self.read(Register::Configuration).await?;
        Ok(Configuration::from(u16::from_be_bytes(data)))
    }

    /// Set configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_configuration(&mut self, config: Configuration) -> Result<(), I2C::Error> {
        let value: u16 = config.into();
        self.write(Register::Configuration, value.to_be_bytes()).await?;

        // Only cache config after successful write
        self.config = config;
        Ok(())
    }

    /// Read temperature register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
        self.delay.delay_ms(Self::CONVERSION_TIME_TYPICAL_MS).await;
        let raw = self.read(Register::Temperature).await?;
        Ok(Self::to_celsius(i16::from_be_bytes(raw)))
    }

    /// Configure device for One-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.set_mode(ConversionMode::Continuous).await
    }

    /// Place device in Shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.set_mode(ConversionMode::Shutdown).await
    }

    /// Initiate continuous conversions
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn continuous<F, Fut>(&mut self, mut config: Configuration, f: F) -> Result<(), I2C::Error>
    where
        F: FnOnce(&mut Self) -> Fut,
        Fut: Future<Output = Result<(), I2C::Error>> + Send,
    {
        config.set_cm(ConversionMode::Continuous);
        self.set_configuration(config).await?;
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
    pub async fn wait_for_temperature(&mut self) -> Result<f32, I2C::Error> {
        let config = self.configuration().await?;

        let delay = match config.cr() {
            ConversionRate::Hertz025 => 4_000_000,
            ConversionRate::Hertz1 => 1_000_000,
            ConversionRate::Hertz4 => 250_000,
            ConversionRate::Hertz16 => 62_500,
        };

        self.delay.delay_us(delay).await;
        self.temperature().await
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.read(Register::LowLimit).await?;
        Ok(Self::to_celsius(i16::from_be_bytes(raw)))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = Self::to_raw(limit);
        self.write(Register::LowLimit, raw.to_be_bytes()).await
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.read(Register::HighLimit).await?;
        Ok(Self::to_celsius(i16::from_be_bytes(raw)))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = Self::to_raw(limit);
        self.write(Register::HighLimit, raw.to_be_bytes()).await
    }

    async fn set_mode(&mut self, mode: ConversionMode) -> Result<(), I2C::Error> {
        let mut config = self.configuration().await?;
        config.set_cm(mode);
        self.set_configuration(config).await
    }

    async fn read(&mut self, reg: Register) -> Result<[u8; 2], I2C::Error> {
        let mut bytes = [0; 2];
        self.i2c.write_read(self.addr, &[reg.into()], &mut bytes).await?;
        Ok(bytes)
    }

    async fn write(&mut self, reg: Register, value: [u8; 2]) -> Result<(), I2C::Error> {
        let mut data = [0; 3];

        data[0] = reg.into();
        data[1..].copy_from_slice(&value);

        self.i2c.write(self.addr, &data).await
    }

    fn to_celsius(t: i16) -> f32 {
        f32::from(t / 16) * Self::CELSIUS_PER_BIT
    }

    #[allow(clippy::cast_possible_truncation)]
    fn to_raw(t: f32) -> i16 {
        (t * 16.0 / Self::CELSIUS_PER_BIT) as i16
    }
}

/// Tmp108 Errors
#[derive(Debug)]
pub enum Error<E: embedded_hal_async::i2c::Error> {
    /// I2C Bus Error
    Bus(E),
    /// Invalid Input Error
    InvalidInput,
    /// Other Error
    Other,
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<E: embedded_hal_async::i2c::Error> sensor::Error for Error<E> {
    fn kind(&self) -> sensor::ErrorKind {
        match *self {
            Self::Bus(_) => sensor::ErrorKind::Peripheral,
            Self::InvalidInput => sensor::ErrorKind::InvalidInput,
            Self::Other => sensor::ErrorKind::Other,
        }
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> sensor::ErrorType
    for Tmp108<I2C, DELAY>
{
    type Error = Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> TemperatureSensor
    for Tmp108<I2C, DELAY>
{
    async fn temperature(&mut self) -> Result<DegreesCelsius, Self::Error> {
        self.temperature().await.map_err(Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> TemperatureThresholdSet
    for Tmp108<I2C, DELAY>
{
    async fn set_temperature_threshold_low(&mut self, threshold: DegreesCelsius) -> Result<(), Self::Error> {
        self.set_low_limit(threshold).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(&mut self, threshold: DegreesCelsius) -> Result<(), Self::Error> {
        self.set_high_limit(threshold).await.map_err(Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> TemperatureHysteresis
    for Tmp108<I2C, DELAY>
{
    async fn set_temperature_threshold_hysteresis(&mut self, hysteresis: DegreesCelsius) -> Result<(), Self::Error> {
        /* Trait method takes a continuous range of f32 values as argument,
         * but internally driver only accepts 4 discrete values for hysteresis.
         * We ensure only a correct value for hysteresis is passed in, and return error otherwise.
         */
        let hysteresis = if (hysteresis - 0.0).abs() < f32::EPSILON {
            Hysteresis::ZeroCelsius
        } else if (hysteresis - 1.0).abs() < f32::EPSILON {
            Hysteresis::OneCelsius
        } else if (hysteresis - 2.0).abs() < f32::EPSILON {
            Hysteresis::TwoCelsius
        } else if (hysteresis - 4.0).abs() < f32::EPSILON {
            Hysteresis::FourCelsius
        } else {
            return Err(Error::InvalidInput);
        };

        let config = self.config.with_hysteresis(hysteresis);
        self.set_configuration(config).await.map_err(Error::Bus)
    }
}

/// TMP108 asynchronous device driver (with alert pin)
#[cfg(feature = "embedded-sensors-hal-async")]
pub struct AlertTmp108<
    I2C: embedded_hal_async::i2c::I2c,
    DELAY: embedded_hal_async::delay::DelayNs,
    ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
> {
    /// Underlying TMP108 sensor.
    pub tmp108: Tmp108<I2C, DELAY>,
    alert: ALERT,
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    > AlertTmp108<I2C, DELAY, ALERT>
{
    /// Create a new ALERTTMP108 instance.
    pub async fn new_async(i2c: I2C, delay: DELAY, a0: A0, alert: ALERT) -> Self {
        let tmp108 = Tmp108::new_async(i2c, delay, a0).await;
        Self { tmp108, alert }
    }

    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    pub async fn new_async_with_a0_gnd(i2c: I2C, delay: DELAY, alert: ALERT) -> Self {
        Self::new_async(i2c, delay, A0::Gnd, alert).await
    }

    /// Create a new ALERTTMP108 instance with A0 tied to V+, resulting in an
    /// instance responding to address `0x49`.
    pub async fn new_async_with_a0_vplus(i2c: I2C, delay: DELAY, alert: ALERT) -> Self {
        Self::new_async(i2c, delay, A0::Vplus, alert).await
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SDA, resulting in an
    /// instance responding to address `0x4a`.
    pub async fn new_async_with_a0_sda(i2c: I2C, delay: DELAY, alert: ALERT) -> Self {
        Self::new_async(i2c, delay, A0::Sda, alert).await
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SCL, resulting in an
    /// instance responding to address `0x4b`.
    pub async fn new_async_with_a0_scl(i2c: I2C, delay: DELAY, alert: ALERT) -> Self {
        Self::new_async(i2c, delay, A0::Scl, alert).await
    }

    /// Destroy the driver instance, return the I2C bus instance and ALERT pin instance.
    pub fn destroy(self) -> (I2C, ALERT) {
        (self.tmp108.destroy(), self.alert)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    > sensor::ErrorType for AlertTmp108<I2C, DELAY, ALERT>
{
    type Error = Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    > TemperatureSensor for AlertTmp108<I2C, DELAY, ALERT>
{
    async fn temperature(&mut self) -> Result<DegreesCelsius, Self::Error> {
        self.tmp108.temperature().await.map_err(Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    > TemperatureThresholdSet for AlertTmp108<I2C, DELAY, ALERT>
{
    async fn set_temperature_threshold_low(&mut self, threshold: DegreesCelsius) -> Result<(), Self::Error> {
        self.tmp108.set_low_limit(threshold).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(&mut self, threshold: DegreesCelsius) -> Result<(), Self::Error> {
        self.tmp108.set_high_limit(threshold).await.map_err(Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    > TemperatureThresholdWait for AlertTmp108<I2C, DELAY, ALERT>
{
    async fn wait_for_temperature_threshold(&mut self) -> Result<DegreesCelsius, Self::Error> {
        match (self.tmp108.config.tm(), self.tmp108.config.polarity()) {
            // In comparator mode, the ALERT pin remains active even after triggering.
            // If called in a loop, next iteration would return immediately (after reading config again) if temperature remains outside threshold.
            // ALERT pin only resets when temperature falls within the range of (Tlow + HYS) and (Thigh - HYS).
            (ThermostatMode::Comparator, Polarity::ActiveLow) => {
                self.alert.wait_for_low().await.map_err(|_| Error::Other)?;
            }
            (ThermostatMode::Comparator, Polarity::ActiveHigh) => {
                self.alert.wait_for_high().await.map_err(|_| Error::Other)?;
            }

            // In interrupt mode, the ALERT pin is immediately reset (by reading config register) after triggering.
            // If called in a loop, next iteration would wait even if temperature remains outside threshold.
            (ThermostatMode::Interrupt, Polarity::ActiveLow) => {
                self.alert.wait_for_falling_edge().await.map_err(|_| Error::Other)?;
                let _ = self.tmp108.configuration().await.map_err(Error::Bus)?;
            }
            (ThermostatMode::Interrupt, Polarity::ActiveHigh) => {
                self.alert.wait_for_rising_edge().await.map_err(|_| Error::Other)?;
                let _ = self.tmp108.configuration().await.map_err(Error::Bus)?;
            }
        }

        // Return temperature at time of trigger for caller to determine which threshold was crossed.
        let temperature = self.tmp108.temperature().await.map_err(Error::Bus)?;
        Ok(temperature)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    > TemperatureHysteresis for AlertTmp108<I2C, DELAY, ALERT>
{
    async fn set_temperature_threshold_hysteresis(&mut self, hysteresis: DegreesCelsius) -> Result<(), Self::Error> {
        self.tmp108.set_temperature_threshold_hysteresis(hysteresis).await
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;
    use crate::{Hysteresis, Polarity, ThermostatMode};

    #[tokio::test]
    async fn handle_a0_pin_accordingly() {
        let expectations = vec![];

        let mock = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let tmp = Tmp108::new_async_with_a0_gnd(mock, delay).await;
        assert_eq!(tmp.addr, 0x48);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let tmp = Tmp108::new_async_with_a0_vplus(mock, delay).await;
        assert_eq!(tmp.addr, 0x49);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let tmp = Tmp108::new_async_with_a0_sda(mock, delay).await;
        assert_eq!(tmp.addr, 0x4a);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let tmp = Tmp108::new_async_with_a0_scl(mock, delay).await;
        assert_eq!(tmp.addr, 0x4b);
        let mut mock = tmp.destroy();
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
            let delay = NoopDelay::new();
            let mut tmp108 = Tmp108::new_async_with_a0_gnd(mock, delay).await;
            let result = tmp108.temperature().await;
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_approx_eq!(temp, *t, 1e-4);

            let mut mock = tmp108.destroy();
            mock.done();
        }
    }

    #[tokio::test]
    async fn read_write_configuration_register() {
        let expectations = vec![
            Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
            Transaction::write(0x48, vec![0x01, 0xfe, 0xb0]),
        ];

        let mock = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let mut tmp = Tmp108::new_async_with_a0_gnd(mock, delay).await;
        let result = tmp.configuration().await;
        assert!(result.is_ok());

        let cfg = result.unwrap();
        assert_eq!(cfg, Configuration::default());

        let cfg = cfg
            .with_cm(ConversionMode::Continuous)
            .with_tm(ThermostatMode::Interrupt)
            .with_fl(true)
            .with_fh(true)
            .with_cr(ConversionRate::Hertz16)
            .with_id(true)
            .with_hysteresis(Hysteresis::FourCelsius)
            .with_polarity(Polarity::ActiveHigh);

        let result = tmp.set_configuration(cfg).await;
        assert!(result.is_ok());

        let mut mock = tmp.destroy();
        mock.done();
    }

    #[cfg(feature = "embedded-sensors-hal-async")]
    #[tokio::test]
    async fn handle_threshold_alerts_properly() {
        use embedded_hal_mock::eh1::digital;

        // Sensor i2c bus mocks and expectations
        let i2c_expectations = vec![
            Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
            Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
            Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
            Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00]),
        ];
        let i2c_mock = Mock::new(&i2c_expectations);
        let delay = NoopDelay::new();

        // Threshold alert GPIO pin mocks and expectations
        let pin_expectations = vec![digital::Transaction::wait_for_edge(digital::Edge::Falling)];
        let pin_mock = digital::Mock::new(&pin_expectations);

        // Create a ALERTTMP108 instance and configure it as active-low interrupt mode
        let mut tmp108 = AlertTmp108::new_async_with_a0_gnd(i2c_mock, delay, pin_mock).await;
        let cfg = Configuration::default()
            .with_tm(ThermostatMode::Interrupt)
            .with_polarity(Polarity::ActiveLow);
        let result = tmp108.tmp108.set_configuration(cfg).await;
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
