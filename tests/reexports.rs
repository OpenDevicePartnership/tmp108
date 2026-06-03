//! Compile-only test: Config field types must be reachable from outside the crate.
use tmp108::{Config, ConversionRate, Hysteresis, Mode, Polarity, Thermostat};

#[test]
fn config_with_explicit_fields_compiles() {
    let cfg = Config {
        thermostat_mode: Thermostat::Interrupt,
        alert_polarity: Polarity::ActiveHigh,
        conversion_rate: ConversionRate::_16Hz,
        hysteresis: Hysteresis::_4C,
    };
    // Avoid unused-binding warning; the assertion is on compilation.
    let _ = cfg;
    // Also pin Mode (re-exported even though not a Config field).
    let _ = Mode::Continuous;
}
