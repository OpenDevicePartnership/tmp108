//! TMP108 temperature read example.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;
use tmp108::Tmp108;

#[cfg(not(feature = "async"))]
fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");

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
