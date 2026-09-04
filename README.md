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

let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);

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

tmp.sensor_mut()
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

| Feature | Effect | Implies |
|---------|--------|---------|
| (none)  | Blocking `Tmp108` over `embedded-hal`. | — |
| `async` | Async `AsyncTmp108` over `embedded-hal-async`. `AsyncTmp108::continuous` is unlocked. | — |
| `embedded-sensors-hal` | Blocking `TemperatureSensor` impl on `Tmp108`. | — |
| `embedded-sensors-hal-async` | Async `TemperatureSensor`, `TemperatureThresholdSet`, `TemperatureHysteresis` impls on `AsyncTmp108`, plus the `AlertTmp108` wrapper with `TemperatureThresholdWait`. | `async` |

Since 0.7.0 both `Tmp108` (blocking) and `AsyncTmp108` (async) are
available simultaneously when both relevant features are enabled.

## Gotchas

- **Constructors are infallible.** They take no delay; the delay only
  appears on `wait_for_temperature`, where it is genuinely needed to
  wait out a conversion period.
- **Comparator vs interrupt mode latching.** In comparator mode the ALERT pin
  stays asserted until temperature returns inside `(T_low + HYS, T_high − HYS)`.
  In interrupt mode the pin clears as soon as the configuration register is
  read (the driver does this for you inside `wait_for_temperature_threshold`).
  See `examples/alert_comparator.rs` for a demonstration.
- **ALERT polarity is set on-chip.** Wire your pull resistor for the polarity
  you configured. Examples assume active-low + external pull-up.
- **`AsyncTmp108::continuous` is async-only.** For blocking continuous-mode
  use, call `Tmp108::configure(...)` to set `Mode::Continuous` manually,
  loop on `Tmp108::wait_for_temperature(&mut delay)`, then call
  `Tmp108::shutdown()`.
- **`AsyncTmp108::continuous` future is not cancel-safe.** Dropping it
  before completion (e.g. via `embassy_futures::select!` or
  `tokio::time::timeout`) leaves the chip in `Mode::Continuous`. See
  the method's doc.
- **Temperature scale.** Raw register values are 12-bit signed in the upper
  bits of a 16-bit register at 0.0625 °C/LSB. The driver models this as
  `Celsius`, a newtype over sixteenths of a degree whose 4096 inhabitants
  are exactly the temperatures the part can report or accept as a limit.
  `Celsius::try_from_degrees` is the single fallible parse (it rejects
  `NaN`, infinities and out-of-range values, and rounds half away from
  zero); `Celsius::to_degrees` renders back to `f32`, and `Display`
  honours precision so `{t:.2}` works. The `embedded-sensors-hal` trait
  impls keep their `DegreesCelsius` (`f32`) signatures and translate at
  the boundary.

## MSRV

Rust 1.94 and up.

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full contribution workflow,
including the [Conventional Commits](https://www.conventionalcommits.org/)
v1.0.0 commit-message format this repository uses.
