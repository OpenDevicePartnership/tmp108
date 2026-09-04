//! Compile-only test: Config field types and driver types must be
//! reachable from outside the crate.
use tmp108::{Config, ConversionRate, Hysteresis, Mode, Polarity, Thermostat};

#[test]
fn config_with_explicit_fields_compiles() {
    let cfg = Config {
        thermostat_mode: Thermostat::Interrupt,
        alert_polarity: Polarity::ActiveHigh,
        conversion_rate: ConversionRate::SixteenHz,
        hysteresis: Hysteresis::FourC,
    };
    // Avoid unused-binding warning; the assertion is on compilation.
    let _ = cfg;
    // Also pin Mode (re-exported even though not a Config field).
    let _ = Mode::Continuous;
}

/// The blocking driver type must be reachable from outside the crate.
#[allow(dead_code)]
fn pin_blocking_driver_type() {
    fn _type_check<I2C: embedded_hal::i2c::I2c>(_: tmp108::Tmp108<I2C>) {}
}

/// The async driver type must be reachable from outside the crate when
/// the `async` feature is enabled.
#[cfg(feature = "async")]
#[allow(dead_code)]
fn pin_async_driver_type() {
    fn _type_check<I2C: embedded_hal_async::i2c::I2c>(_: tmp108::AsyncTmp108<I2C>) {}
}
