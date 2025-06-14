#![allow(missing_docs)]
use bilge::prelude::*;

/// Register addresses
#[derive(Debug, PartialEq, PartialOrd)]
pub enum Register {
    /// Temperature register address.
    Temperature,

    /// Configuration register address.
    Configuration,

    /// Temperature low limit register address.
    LowLimit,

    /// Temperature high limit register address.
    HighLimit,
}

impl From<Register> for u8 {
    fn from(reg: Register) -> Self {
        match reg {
            Register::Temperature => 0,
            Register::Configuration => 1,
            Register::LowLimit => 2,
            Register::HighLimit => 3,
        }
    }
}

/// Temperature register.
#[bitsize(16)]
#[derive(DebugBits)]
pub struct Temperature(u16);

/// Temperature high limit register.
#[bitsize(16)]
#[derive(DebugBits)]
pub struct HighLimit(u16);

/// Temperature low limit register.
#[bitsize(16)]
#[derive(DebugBits)]
pub struct LowLimit(u16);

/// Configuration register.
#[bitsize(16)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct Configuration {
    reserved0_3: u4,

    /// Hysteresis control.
    pub hysteresis: Hysteresis,

    reserved6: bool,

    /// Polarity
    pub polarity: Polarity,

    /// Conversion mode
    pub cm: ConversionMode,

    /// Thermostat mode
    pub tm: ThermostatMode,

    /// Flag low
    pub fl: bool,

    /// Flag high
    pub fh: bool,

    /// Conversion rate
    pub cr: ConversionRate,

    /// ID bit
    pub id: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self::from(0b0010_0010_0001_0000)
    }
}

impl Configuration {
    /// Configure conversion mode.
    #[must_use]
    pub fn with_cm(mut self, mode: ConversionMode) -> Self {
        self.set_cm(mode);
        Self::from(self.value)
    }

    /// Configure thermostat mode.
    #[must_use]
    pub fn with_tm(mut self, mode: ThermostatMode) -> Self {
        self.set_tm(mode);
        Self::from(self.value)
    }

    /// Configure flag low.
    #[must_use]
    pub fn with_fl(mut self, flag: bool) -> Self {
        self.set_fl(flag);
        Self::from(self.value)
    }

    /// Configure flag high.
    #[must_use]
    pub fn with_fh(mut self, flag: bool) -> Self {
        self.set_fh(flag);
        Self::from(self.value)
    }

    /// Configure conversion rate.
    #[must_use]
    pub fn with_cr(mut self, rate: ConversionRate) -> Self {
        self.set_cr(rate);
        Self::from(self.value)
    }

    /// Configure ID bit.
    #[must_use]
    pub fn with_id(mut self, id: bool) -> Self {
        self.set_id(id);
        Self::from(self.value)
    }

    /// Configure hysteresis.
    #[must_use]
    pub fn with_hysteresis(mut self, hyst: Hysteresis) -> Self {
        self.set_hysteresis(hyst);
        Self::from(self.value)
    }

    /// Configure alert polarity.
    #[must_use]
    pub fn with_polarity(mut self, polarity: Polarity) -> Self {
        self.set_polarity(polarity);
        Self::from(self.value)
    }
}

/// Conversion mode.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum ConversionMode {
    /// Shutdown all device circuitry other than the serial interface. Current
    /// consumption is typically less than 0.5uA while in this mode.
    Shutdown,

    /// One-shot conversion. Started from `Shutdown` mode, causes the device to
    /// initiate a single temperature conversion. Once completed, device goes
    /// back to shutdown state.
    OneShot,

    /// Continuous conversion. Periodic conversions at the rate indicated by
    /// `ConversionRate` bits (CR1 and CR0 in the Configuration register).
    #[fallback]
    Continuous,
}

/// Thermostat mode.
#[bitsize(1)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum ThermostatMode {
    /// Comparator mode for the thermostat.
    Comparator,

    /// Interrupt mode for the thermostat.
    Interrupt,
}

/// Conversion rate.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum ConversionRate {
    /// 0.25Hz conversion rate.
    Hertz025,

    /// 1Hz conversion rate.
    Hertz1,

    /// 4Hz conversion rate.
    Hertz4,

    /// 16Hz conversion rate.
    Hertz16,
}

/// Hysteresis control.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum Hysteresis {
    /// 0℃ Hysteresis
    ZeroCelsius,

    /// 1℃ Hysteresis
    OneCelsius,

    /// 2℃ Hysteresis
    TwoCelsius,

    /// 4℃ Hysteresis
    FourCelsius,
}

/// Polarity
#[bitsize(1)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum Polarity {
    /// Active low (default).
    ActiveLow,

    /// Active high.
    ActiveHigh,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_configuration() {
        let cfg = Configuration::default();
        assert_eq!(cfg.value, 0x2210);
    }

    #[test]
    fn modify_conversion_mode() {
        let cfg = Configuration::default().with_cm(ConversionMode::Shutdown);
        assert_eq!(cfg.value, 0x2010);
    }

    #[test]
    fn modify_thermostat_mode() {
        let cfg = Configuration::default().with_tm(ThermostatMode::Interrupt);
        assert_eq!(cfg.value, 0x2610);
    }

    #[test]
    fn modify_flag_low() {
        let cfg = Configuration::default().with_fl(true);
        assert_eq!(cfg.value, 0x2a10);
    }

    #[test]
    fn modify_flag_high() {
        let cfg = Configuration::default().with_fh(true);
        assert_eq!(cfg.value, 0x3210);
    }

    #[test]
    fn modify_conversion_rate() {
        let cfg = Configuration::default().with_cr(ConversionRate::Hertz16);
        assert_eq!(cfg.value, 0x6210);
    }

    #[test]
    fn modify_id() {
        let cfg = Configuration::default().with_id(true);
        assert_eq!(cfg.value, 0xa210);
    }

    #[test]
    fn modify_hysteresis() {
        let cfg = Configuration::default().with_hysteresis(Hysteresis::FourCelsius);
        assert_eq!(cfg.value, 0x2230);
    }

    #[test]
    fn modify_polarity() {
        let cfg = Configuration::default().with_polarity(Polarity::ActiveHigh);
        assert_eq!(cfg.value, 0x2290);
    }
}
