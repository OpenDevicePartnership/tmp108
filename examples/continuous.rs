//! TMP108 continuous-conversion example.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//!
//! # Cargo features
//!
//! This example is **async only**. Requires `--features async`. The blocking
//! `Tmp108` API does not expose a `continuous` closure helper; if you need
//! continuous-mode conversions in a blocking context, call `configure(...)`
//! to set the M bits to `Mode::Continuous` manually, loop on
//! `wait_for_temperature(&mut delay)`, then call `shutdown()` when done.
//!
//! # Register interactions
//!
//! 1. Read configuration (modify-cycle for M bits)
//! 2. Write configuration with M=Continuous
//! 3. Read configuration (used by `wait_for_temperature` to determine delay)
//! 4. Read temperature register (per iteration)
//! 5. On closure return: read & rewrite configuration with M=Shutdown

#[cfg(not(feature = "async"))]
fn main() {
    eprintln!("examples/continuous.rs requires --features async");
}

#[cfg(feature = "async")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use pico_de_gallo_hal::Hal;
    use tmp108::Tmp108;

    // README-SNIPPET-START: continuous
    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut delay = hal.delay();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);

    tmp.continuous(async |t| {
        for _ in 0..5 {
            let temperature = t.wait_for_temperature(&mut delay).await?;
            println!("Temperature: {temperature:.2} C");
        }
        Ok(())
    })
    .await
    .map_err(|_| anyhow!("Continuous conversion failed"))?;
    // README-SNIPPET-END: continuous

    Ok(())
}
