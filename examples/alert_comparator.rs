//! TMP108 ALERT pin example — comparator mode (latched behavior).
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
//! Requires `--features async,embedded-sensors-hal-async`.
//!
//! # Behavior
//!
//! In comparator mode the ALERT pin stays asserted until the temperature
//! returns to within `(T_low + HYS, T_high − HYS)`. This example demonstrates
//! that:
//! - A `wait_for_temperature_threshold` call returns when the threshold is
//!   first crossed.
//! - A *second* call returns *immediately* while the pin is still asserted.
//! - Once the temperature falls back inside the hysteresis band, the pin
//!   releases; the next call blocks again.
//!
//! # Usage
//!
//! Run the example, then warm the TMP108 with a finger to trip the high
//! threshold. Hold the warmth while the program prints two back-to-back
//! readings demonstrating the latched pin, then let it cool below 28 C
//! (high threshold 30 − 2 °C hysteresis) and observe the next call blocks.

#[cfg(not(all(feature = "async", feature = "embedded-sensors-hal-async")))]
fn main() {
    eprintln!("examples/alert_comparator.rs requires --features async,embedded-sensors-hal-async");
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};
    use pico_de_gallo_hal::Hal;
    use pico_de_gallo_lib::{GpioDirection, GpioPull};
    use tmp108::{AlertTmp108, Config, Hysteresis, Polarity, Thermostat};

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut alert = hal.gpio(0);
    alert
        .set_config(GpioDirection::Input, GpioPull::None)
        .map_err(|_| anyhow!("Failed to configure GPIO0 as input"))?;

    let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);

    tmp.tmp108
        .configure(Config {
            thermostat_mode: Thermostat::Comparator,
            alert_polarity: Polarity::ActiveLow,
            hysteresis: Hysteresis::_2C,
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

    println!("Waiting for first ALERT (warm the sensor above 30 C)...");
    let t1 = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("First wait failed"))?;
    println!("First trigger: {t1:.2} C (pin is latched)");

    // While the temperature is still above 30 - 2 = 28 C, this returns
    // immediately because the pin remains asserted.
    let t2 = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("Second wait failed"))?;
    println!("Second trigger (immediate): {t2:.2} C");

    println!("Let the sensor cool below 28 C; the next ALERT will reassert when temperature rises again...");
    let t3 = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("Third wait failed"))?;
    println!("Third trigger: {t3:.2} C");

    Ok(())
}
