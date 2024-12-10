//! Tmp108 Async API
use super::*;
use core::future::Future;

/// TMP108 asynchronous device driver
pub struct Tmp108<I2C: embedded_hal_async::i2c::I2c, DELAY: embedded_hal_async::delay::DelayNs> {
    /// The concrete I2C bus implementation
    i2c: I2C,

    /// The concrete [`embedded_hal::delay::DelayNs`] implementation
    delay: DELAY,

    /// The I2C address.
    pub(crate) addr: u8,
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
        self.write(Register::Configuration, value.to_be_bytes()).await
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

    /// Set temperature low limit register
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
        data[1] = value[0];
        data[2] = value[1];

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

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;

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
        let temps = vec![127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

        for (e, t) in expectations.iter().zip(temps.iter()) {
            let mock = Mock::new(e);
            let delay = NoopDelay::new();
            let mut tmp = Tmp108::new_async_with_a0_gnd(mock, delay).await;
            let result = tmp.temperature().await;
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_approx_eq!(temp, *t, 1e-4);

            let mut mock = tmp.destroy();
            mock.done();
        }
    }

    #[tokio::test]
    async fn read_write_configuration_register() {
        let expectations = vec![
            Transaction::write_read(0x48, vec![0x01], vec![0x10, 0x22]),
            Transaction::write(0x48, vec![0x01, 0xb0, 0xfe]),
        ];

        let mock = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let mut tmp = Tmp108::new_async_with_a0_gnd(mock, delay).await;
        let result = tmp.configuration().await;
        assert!(result.is_ok());

        let cfg = result.unwrap();
        assert_eq!(cfg, Default::default());

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
}
