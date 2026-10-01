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
let mut delay = hal.delay();

let mut tmp = Tmp108::new_with_a0_gnd(i2c);

// 40 ms settling delay: the part defers shutdown until the conversion
// already in progress finishes, so the write alone is not quiescence.
let temperature = tmp
    .one_shot(&mut delay, 40)
    .map_err(|_| anyhow!("Failed to acquire a one-shot conversion"))?;
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
println!("ALERT serviced! Latest temperature: {temperature:.2} C");
```

See `examples/` for complete, runnable versions of each snippet (and more).

## Cargo features

| Feature | Effect | Implies |
|---------|--------|---------|
| (none)  | Blocking `Tmp108` over `embedded-hal`. | — |
| `async` | Async `AsyncTmp108` over `embedded-hal-async`. `AsyncTmp108::continuous` is unlocked. | — |
| `embedded-sensors-hal` | Blocking `TemperatureSensor` impl on `Tmp108`. | — |
| `embedded-sensors-hal-async` | Async `TemperatureSensor`, `TemperatureThresholdSet`, `TemperatureHysteresis` impls on `AsyncTmp108`, plus the `AlertTmp108` wrapper with `TemperatureThresholdWait`. | `async` |

Since 0.6.0 both `Tmp108` (blocking) and `AsyncTmp108` (async) are
available simultaneously when both relevant features are enabled.

## Gotchas

- **Constructors are infallible.** They take no delay; the delay appears on
  the operations that genuinely have to wait — `wait_for_temperature`, to
  wait out a conversion period, and `one_shot`, to let the part settle into
  shutdown and then poll for completion.
- **`one_shot` is the whole acquisition, not a trigger.** It drives the part
  into shutdown, triggers, waits for the chip to clear `M` back to `0b00`,
  and only then reads the temperature register, returning the sample. The
  settling delay is yours to pick: the part defers shutdown until any
  conversion in progress finishes, so the write alone is not quiescence.
- **Comparator vs interrupt mode latching.** In comparator mode the ALERT pin
  stays asserted until temperature returns inside `(T_low + HYS, T_high − HYS)`.
  In interrupt mode the pin clears as soon as the configuration register is
  read (the driver does this for you inside `wait_for_temperature_threshold`).
  See `examples/alert_comparator.rs` for a demonstration.
- **Any configuration read acknowledges, not just the waiter's.** In
  interrupt mode every read of the configuration register clears FL/FH and
  releases the ALERT pin — including `wait_for_temperature()`, which reads
  it only to pick a delay, `configure()` and `shutdown()`, whose
  read-modify-write acknowledges on the read half, and
  `set_temperature_threshold_hysteresis()`, which does it
  twice. A configuration *write* does not acknowledge. Only
  `read_configuration_and_acknowledge()` hands back the raw flag pair;
  `AlertTmp108::wait_for_alert()` reports them as an interpreted
  `AlertCause`, and everything else discards them. See the
  [crate documentation](https://docs.rs/tmp108/latest/tmp108/#interrupt-mode-acknowledgement)
  for the full table.
- **Retained delivery precedes fresh acquisition.** After an acknowledged
  interrupt's temperature read fails or is cancelled, the next threshold
  wait reads temperature once, with no configuration read, GPIO wait, or
  acknowledgment. This delivery obligation survives direct sensor access
  and reconfiguration, even to comparator mode. It is wrapper-local:
  `into_inner()`, `destroy()`, or dropping the wrapper abandons it without
  driver I/O; re-wrapping starts empty.
- **Fresh acquisition observes pending interrupts before waiting.** With no
  retained delivery, the threshold waiter captures FL/FH from its entry
  configuration read. In interrupt mode, either
  flag makes it proceed directly to the temperature read, without GPIO
  waiting or a second acknowledgment. With neither flag set, it waits for
  the asserted pin level, not an edge, then acknowledges the alert.
- **An alert reading is not a trigger-time sample.** Both waiters return
  the latest conversion, read after observing an alert. It may be back
  inside the configured band. A retained delivery is not a cached sample:
  it reads at retry time, with no bound on the time since the crossing or
  the age of the register's conversion.
- **Only `wait_for_alert` reports the cause.** The scalar
  `wait_for_temperature_threshold` returns a temperature alone and cannot
  identify whether FL, FH, or both caused the event.
  `AlertTmp108::wait_for_alert` returns an `AlertEvent` carrying an
  `AlertCause`, but a fresh comparator-mode acquisition always reports
  `Unknown`, as does interrupt mode when the acknowledging read finds both
  flags clear. Both methods draw on one delivery obligation: a successful
  scalar delivery discards the cause.
- **The threshold waiter is not event-delivery cancel-safe.** A configuration
  read may consume an interrupt before returning success. Failure or
  cancellation during the entry or acknowledging read can therefore lose
  it unrecoverably. Only the temperature stage after successful interrupt
  acknowledgment retains delivery for a later call. `Error::Bus` does not
  identify the failed stage or prove that no alert occurred. This is not
  exactly-once delivery: relatching or sustained comparator assertion can
  produce multiple successful calls for one excursion. See the
  [`AlertTmp108` documentation](https://docs.rs/tmp108/latest/tmp108/struct.AlertTmp108.html)
  for the full contract.
- **Retries need application-level backoff or a bound.** The driver never
  retries internally. With a retained obligation and an immediately failing
  bus, repeatedly awaiting the waiter returns an immediately-ready `Err`
  each time and need not yield to other tasks.
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
