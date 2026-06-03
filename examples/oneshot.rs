//! TMP108 one-shot conversion example.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//!
//! # Cargo features
//!
//! Works with default features (blocking). Building with `--features async`
//! produces the async variant.
//!
//! # Register interactions
//!
//! Single read of the temperature register at address `0x00`.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;
use tmp108::Tmp108;

#[cfg(not(feature = "async"))]
fn main() -> Result<()> {
    // README-SNIPPET-START: oneshot
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");
    // README-SNIPPET-END: oneshot

    Ok(())
}

#[cfg(feature = "async")]
#[tokio::main]
async fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp
        .temperature()
        .await
        .map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");

    Ok(())
}
