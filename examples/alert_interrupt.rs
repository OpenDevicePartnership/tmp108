//! TMP108 ALERT pin example — interrupt mode.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//! - TMP108 ALERT pin tied to Pico de Gallo GPIO0 with an external 2 kΩ
//!   pull-up to V+
//!
//! # Cargo features
//!
//! Requires `--features async,embedded-sensors-hal-async`. The `AlertTmp108`
//! wrapper that exposes `wait_for_temperature_threshold` only exists under
//! this combination.
//!
//! # Behavior
//!
//! In interrupt mode the ALERT pin pulses (it clears as soon as the
//! configuration register is read). A `wait_for_temperature_threshold` call
//! in interrupt mode returns once, after which the pin is reset; the next
//! call will block until a new threshold crossing occurs.
//!
//! # Usage
//!
//! Run the example, then warm the TMP108 with a finger (or breathe on it) to
//! cross the 30 °C high threshold. The program will print the temperature at
//! the moment of the trigger and exit.

#[cfg(not(all(feature = "async", feature = "embedded-sensors-hal-async")))]
fn main() {
    eprintln!("examples/alert_interrupt.rs requires --features async,embedded-sensors-hal-async");
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};
    use pico_de_gallo_hal::Hal;
    use pico_de_gallo_lib::{GpioDirection, GpioPull};
    use tmp108::{AlertTmp108, Config, Polarity, Thermostat};

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut alert = hal.gpio(0);
    alert
        .set_config(GpioDirection::Input, GpioPull::None)
        .map_err(|_| anyhow!("Failed to configure GPIO0 as input"))?;

    // README-SNIPPET-START: alert
    let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);

    tmp.tmp108
        .configure(Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveLow,
            ..Default::default()
        })
        .await
        .map_err(|_| anyhow!("Failed to configure TMP108"))?;

    tmp.set_temperature_threshold_low(15.0)
        .await
        .map_err(|_| anyhow!("Failed to set low threshold"))?;
    tmp.set_temperature_threshold_high(30.0)
        .await
        .map_err(|_| anyhow!("Failed to set high threshold"))?;

    println!("Waiting for ALERT (warm the sensor above 30 C)...");
    let temperature = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("wait_for_temperature_threshold failed"))?;
    println!("ALERT! Temperature at trigger: {temperature:.2} C");
    // README-SNIPPET-END: alert

    Ok(())
}
