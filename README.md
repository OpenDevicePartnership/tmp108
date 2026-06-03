# TMP108

[![no-std](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/nostd.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/nostd.yml)
[![check](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/check.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/check.yml)
[![rolling](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/rolling.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/rolling.yml)
[![crates.io](https://img.shields.io/crates/v/tmp108.svg)](https://crates.io/crates/tmp108)
[![Documentation](https://docs.rs/tmp108/badge.svg)](https://docs.rs/tmp108)
[![LICENSE](https://img.shields.io/badge/License-MIT-blue)](./LICENSE)

A `#[no_std]` platform-agnostic driver for the
[TMP108](https://www.ti.com/lit/gpn/tmp108) temperature sensor using
the [embedded-hal](https://docs.rs/embedded-hal) traits.

## I²C addresses

The TMP108 can take one of 4 I²C addresses depending on the state of
the A0 pin:

| A0  | Addr |
|-----|------|
| GND | 0x48 |
| V+  | 0x49 |
| SDA | 0x4a |
| SCL | 0x4b |

The driver has dedicated constructors for each — `new_with_a0_gnd`,
`new_with_a0_vplus`, `new_with_a0_sda`, `new_with_a0_scl` — so an invalid
address cannot be requested.

## Usage

### Blocking one-shot read

<!-- snippet: oneshot -->
```rust,ignore
let hal = Hal::new();
let i2c = hal.i2c();

let mut tmp = Tmp108::new_with_a0_gnd(i2c);
let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
println!("Temperature: {temperature:.2} C");
```

### Async continuous conversions

<!-- snippet: continuous -->
```rust,ignore
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
```

### Async interrupt-mode threshold alert

<!-- snippet: alert -->
```rust,ignore
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
```

See `examples/` for complete, runnable versions of each snippet (and more).

## Cargo features

| Feature | Effect | Requires |
|---------|--------|----------|
| (none)  | Blocking `Tmp108` over `embedded-hal`. | — |
| `async` | Async `Tmp108` over `embedded-hal-async`. `Tmp108::continuous` is unlocked. | — |
| `embedded-sensors-hal` | Blocking `TemperatureSensor` impl on `Tmp108`. | — |
| `embedded-sensors-hal-async` | Async `TemperatureSensor`, `TemperatureThresholdSet`, `TemperatureHysteresis` impls on `Tmp108`, plus the `AlertTmp108` wrapper with `TemperatureThresholdWait`. | `async` |

## Gotchas

- **`Tmp108::new` does not take a delay.** The delay only appears on
  `wait_for_temperature`, where it is genuinely needed to wait out a
  conversion period.
- **Comparator vs interrupt mode latching.** In comparator mode the ALERT pin
  stays asserted until temperature returns inside `(T_low + HYS, T_high − HYS)`.
  In interrupt mode the pin clears as soon as the configuration register is
  read (the driver does this for you inside `wait_for_temperature_threshold`).
  See `examples/alert_comparator.rs` for a demonstration.
- **ALERT polarity is set on-chip.** Wire your pull resistor for the polarity
  you configured. Examples assume active-low + external pull-up.
- **`Tmp108::continuous` is async-only.** For blocking continuous-mode use,
  call `configure(...)` to set `Mode::Continuous` manually, loop on
  `wait_for_temperature(&mut delay)`, then call `shutdown()`.
- **Temperature scale.** Raw register values are 12-bit signed in the upper
  bits of a 16-bit register; the driver returns `f32` Celsius at
  0.0625 °C/LSB.

## MSRV

Rust 1.90 and up.

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full contribution workflow,
including the [Conventional Commits](https://www.conventionalcommits.org/)
v1.0.0 commit-message format this repository uses.
