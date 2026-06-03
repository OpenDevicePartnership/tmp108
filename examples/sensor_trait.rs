//! TMP108 used through the `embedded-sensors-hal` traits.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//!
//! # Cargo features
//!
//! - Blocking variant: `--features embedded-sensors-hal`.
//! - Async variant: `--features async,embedded-sensors-hal-async`.
//!
//! Demonstrates that downstream firmware can call `Tmp108` through the
//! generic `TemperatureSensor` trait without naming the concrete sensor type.
//! This is the pattern to follow when your firmware needs to be parametric
//! over a set of temperature sensors.

#[cfg(all(not(feature = "async"), feature = "embedded-sensors-hal"))]
fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal::temperature::{DegreesCelsius, TemperatureSensor};
    use pico_de_gallo_hal::Hal;
    use tmp108::Tmp108;

    fn read_any_sensor<S: TemperatureSensor>(sensor: &mut S) -> Result<DegreesCelsius, S::Error> {
        sensor.temperature()
    }

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut tmp = Tmp108::new_with_a0_gnd(i2c);

    let temperature = read_any_sensor(&mut tmp).map_err(|_| anyhow!("read failed"))?;
    println!("Temperature (via trait): {temperature:.2} C");
    Ok(())
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{DegreesCelsius, TemperatureSensor};
    use pico_de_gallo_hal::Hal;
    use tmp108::Tmp108;

    async fn read_any_sensor<S: TemperatureSensor>(sensor: &mut S) -> Result<DegreesCelsius, S::Error> {
        sensor.temperature().await
    }

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut tmp = Tmp108::new_with_a0_gnd(i2c);

    let temperature = read_any_sensor(&mut tmp).await.map_err(|_| anyhow!("read failed"))?;
    println!("Temperature (via trait): {temperature:.2} C");
    Ok(())
}

#[cfg(not(any(
    all(not(feature = "async"), feature = "embedded-sensors-hal"),
    all(feature = "async", feature = "embedded-sensors-hal-async"),
)))]
fn main() {
    eprintln!(
        "examples/sensor_trait.rs requires either --features embedded-sensors-hal \
         or --features async,embedded-sensors-hal-async"
    );
}
