# Changelog

All notable changes to this project are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.7.0](https://github.com/OpenDevicePartnership/tmp108/compare/v0.6.0...v0.7.0) - 2026-09-25

### Added

- supervise a complete one-shot acquisition
- *(alert)* report which threshold was crossed

### Fixed

- [**breaking**] make one_shot() acquire a sample, not just trigger one
- do not retrigger a one-shot on reconfiguration
- [**breaking**] decode M = 0b11 as continuous mode
- *(alert)* retain an acknowledged event when the sample read fails
- *(alert)* report an already-latched alert
- *(ddsl)* declare power-on reset values for THIGH and TLOW

### Other

- make the one-shot example actually perform a one-shot
- stop pointing callers at a bare one_shot for fresh reads
- Bump embedded-sensors-hal-async from 0.3.0 to 0.4.0
- Bump tokio from 1.52.3 to 1.53.1
- Bump embedded-sensors-hal from 0.1.0 to 0.1.1
- *(readme)* scope the alert-cause caveat to the scalar waiter
- Update comments in apply_config for clarity
- give the empty-GPIO assertions their reasons
- record async timeline events on poll, not construction
- *(alert)* document retained delivery after a failed sample read
- correct the documented alert contract
- Bump pico de gallo dependencies

### Breaking

- `Mode` decoding is now total (#62). Both `M = 0b10` and `M = 0b11`
  decode as `Mode::Continuous`; previously `0b11` was rejected. TMP108
  datasheet [SBOS663A](https://www.ti.com/lit/gpn/tmp108) §7.4.3,
  "Continuous Conversion Mode (M1 = 1)", defines continuous conversion
  by M1 alone, regardless of M0. Confirmed on silicon with a Pico de
  Gallo and TMP108 at `0x48`: `M = 0b11` converts and reads back as `11`,
  using FH as the conversion detector and shutdown as the control.
  This supersedes the 0.6.0 entry's characterisation of `0b11` as
  "reserved"; that entry records the rejection test shipped at the time.

  **Public API delta from 0.6.0:**
  - `From<u8> for Mode` is added: `0` maps to `Shutdown`, `1` to
    `OneShot`, and every other `u8` value to `Continuous`.
  - `Default for Mode` is added and returns `Continuous`, consistent
    with the part's power-on reset value `0x1022` (`M = 0b10`).
  - The explicit `TryFrom<u8> for Mode` implementation is removed.
    `Mode::try_from` remains available through the standard blanket
    implementation, but `<Mode as TryFrom<u8>>::Error` changes from
    `device_driver::ConversionError<u8>` to `core::convert::Infallible`.
    Decoding cannot fail: there is no conversion error to handle.
  - Encoding is unchanged: `From<Mode> for u8` still encodes
    `Mode::Continuous` as canonical `0b10`, not `0b11`.

  **Migration:** use `Mode::from(raw)` instead of fallible conversion
  and remove conversion-error handling. These are alternative helper
  definitions before and after the change:

  ```rust
  // Before (0.6.0; naming this error requires a direct device-driver dependency)
  fn decode_mode(raw: u8) -> Result<tmp108::Mode, device_driver::ConversionError<u8>> {
      tmp108::Mode::try_from(raw)
  }
  ```

  ```rust
  // After: total conversion, with no error branch
  fn decode_mode(raw: u8) -> tmp108::Mode {
      tmp108::Mode::from(raw)
  }
  ```

  Callers retaining `TryFrom` must update any explicit error type or
  associated-type bound to `core::convert::Infallible`. `ConversionError`
  belongs to `device_driver` and is not re-exported by `tmp108`; downstream
  code naming it directly already needed a direct `device-driver`
  dependency. The generated `Configuration::m()` is not downstream-public
  because `mod inner` is private; the conversion change is public through
  the re-exported `Mode` type. `Polarity`, `Hysteresis`, `ConversionRate`
  and `Thermostat` retain their fallible `TryFrom` implementations and
  are unaffected.

### Added

- Supervised single-sample acquisition through
  `acquire_one_shot(&mut self, delay, shutdown_settle_ms: u32)` on both
  `Tmp108` and `AsyncTmp108` (#60). The result is
  `Result<Celsius, OneShotError<I2C::Error>>`; the new `OneShotError<E>`
  distinguishes `Bus(E)`, `PreparationNotShutdown(Mode)`,
  `UnexpectedMode(Mode)` and `Timeout`. This public API change is
  **purely additive**: existing signatures and `Error<E, P>` are unchanged.

  The helper requests shutdown, waits the caller-supplied settling delay,
  requires a configuration re-read to report `M = 0b00`, then triggers a
  one-shot. Each of up to eight completion polls delays 5 ms before reading:
  `0b01` keeps polling, `0b00` completes, and `0b10` or `0b11` aborts with
  `UnexpectedMode(Mode::Continuous)`. Temperature is read exactly once,
  only after observed completion. Success deliberately leaves the part in
  shutdown; the previous mode is not restored. There is no error-path
  cleanup write, so an error is not a guarantee of shutdown.

  **The settling delay is the caller's responsibility.** TMP108 datasheet
  [SBOS663A](https://www.ti.com/lit/gpn/tmp108) §7.4.1 says the device
  "shuts down when current conversion is completed". Shutdown is deferred:
  a successful write, or a re-read of the `M = 0b00` software just wrote,
  does not prove quiescence. The Electrical Characteristics table's
  21 / 27 / 33 ms conversion-time figures apply at +25 °C and V+ = +1.8 V.
  A 40 ms settling delay is a **starting point, not a guarantee**; validate
  it against the board, supply and temperature range. The driver does not
  validate this argument or infer a safe bound.

  **Interrupt-destructive:** the full sequence performs up to 11
  configuration reads. In interrupt thermostat mode each acknowledges
  FL/FH and releases ALERT (SBOS663A §7.5.3.4). Uncollected interrupt
  evidence is consumed, and the result does not report that loss. Collect
  pending evidence first, or do not use this sample-only helper where
  evidence must survive acquisition. A non-destructive variant remains
  future work (#65).

  **Migration:** replace a bare trigger plus a fixed delay with the
  supervised helper when a fresh sample is required. These alternative
  blocking call sites assume an existing `Tmp108` named `sensor` and an
  `embedded_hal::delay::DelayNs` named `delay`; the after-call uses a
  board-validated `shutdown_settle_ms`. The before-call is the incomplete
  pattern being replaced, not a recommended acquisition sequence:

  ```rust,ignore
  // Before: neither prepares shutdown nor observes completion.
  use embedded_hal::delay::DelayNs;
  sensor.one_shot().unwrap();
  delay.delay_ms(40);
  let temperature = sensor.temperature().unwrap();

  // After: prepares, settles, triggers, polls, then reads.
  let temperature = sensor
      .acquire_one_shot(&mut delay, shutdown_settle_ms)
      .unwrap();
  ```

  Async callers use an `embedded_hal_async::delay::DelayNs` and await
  `sensor.acquire_one_shot(&mut delay, shutdown_settle_ms)`. Production
  callers handle `OneShotError` rather than the illustrative `unwrap()`.
- Observable alert cause through `AlertTmp108::wait_for_alert` (#67),
  supplying the status-bearing API left open by #58 below. The crate root
  now exposes `AlertCause::{BelowLow, AboveHigh, Both, Unknown}` without
  a feature gate, and `AlertEvent { cause: AlertCause, temperature: Celsius }`
  with `embedded-sensors-hal-async`. The additive inherent method requires
  that same feature; its signature is:

  ```rust
  pub async fn wait_for_alert(
      &mut self,
  ) -> Result<
      AlertEvent,
      Error<I2C::Error, <ALERT as embedded_hal::digital::ErrorType>::Error>,
  >;
  ```

  The part records which limit it crossed in FL/FH; the driver previously
  decoded those flags for control flow but discarded the direction. TMP108
  datasheet SBOS663A §7.5.3.4 specifies that FH is set when temperature
  exceeds THIGH and FL when it falls below TLOW. FH is bit 4 and FL is bit 3
  of the first configuration byte (Table 8). FL alone reports `BelowLow`,
  FH alone `AboveHigh`, and both together `Both`. Interrupt acquisition
  uses the flags from the entry read if either is set; otherwise it uses
  the post-wait acknowledging read. No extra status read is added.

  `Unknown` means an alert was observed but direction is unavailable,
  not that no alert occurred:
  - Fresh Comparator acquisition always reports `Unknown`. It performs
    no acknowledging read, and the entry read's flags predate the level
    observation; attributing them to that observation would be a guess.
  - Interrupt acquisition reports `Unknown` when the post-wait
    acknowledging read returns both flags clear. This still counts as a
    delivered alert; requiring nonzero flags would reintroduce the
    lost-event loop fixed in #59.

  In Interrupt mode, flags describe what happened since they were last
  observed and cleared, subject to reset, not what is true now. The
  temperature is the latest conversion (SBOS663A §7.5.2), **not a
  trigger-time sample**. Comparing it against the limits to infer direction
  is the invalid inference documented in #58. `Both` supplies neither the
  number nor the order of excursions.

  A retained cause may be older than its sample. If an interrupt is
  acknowledged and the temperature read then fails or is cancelled, the
  wrapper still owes delivery. A retry reports the original cause with a
  newly read sample, without configuration reads or GPIO waiting. Another
  excursion may have latched in the opposite direction meanwhile, so the
  sample need not support the reported direction. This is expected, not a
  defect. Fresh Comparator acquisition never retains a delivery obligation.

  `wait_for_temperature_threshold` now delegates to `wait_for_alert` and
  converts `event.temperature` with `.to_degrees()`. The delegation leaves
  its signature, behavior, transaction sequence and error mapping unchanged;
  the #58/#59 fixes below still apply. The methods share one delivery
  obligation: either may acquire it and either may settle it. A successful
  scalar delivery discards the cause; a later `wait_for_alert` cannot
  retrieve it. These are two views of one consumptive stream, not two
  subscribers.

  **Upgrade behavior:** relative to 0.6.0, this API addition is purely
  additive: no existing signature changes and no caller needs to change.
  Adding public API requires a **minor** version bump, which
  `cargo semver-checks` will report. **Migration:** callers wanting direction
  can replace the scalar wait with the inherent method. These alternative
  call sites assume an existing `AlertTmp108` named `sensor` in an async
  function with compatible error propagation and `embedded-sensors-hal-async`
  enabled:

  ```rust
  // Before: temperature only
  use embedded_sensors_hal_async::temperature::TemperatureThresholdWait;
  let temperature: f32 = sensor.wait_for_temperature_threshold().await?;

  // After: cause and latest temperature
  let event = sensor.wait_for_alert().await?;
  let cause: tmp108::AlertCause = event.cause;
  let temperature: tmp108::Celsius = event.temperature;
  ```

### Fixed

- `configure()` no longer retriggers an in-flight one-shot when changing
  unrelated settings (#61). Before, its read-modify-write echoed sampled
  `M = 0b01` back to the chip, reissuing a conversion command. Now
  `ops::apply_config` normalises only that encoding to `0b00` (shutdown).
  Raw `0b00`, `0b10` and `0b11` are preserved bit-for-bit; `0b11` is not
  canonicalised to `0b10`. The datasheet's One-Shot Mode description makes
  `M` a trigger/completion field, not an ordinary setting to preserve.

  **Upgrade behavior:** a caller hand-rolling a one-shot **must not call
  `configure()` between triggering the conversion and checking for
  completion**. It now stands the trigger down, so a later `M = 0b00`
  cannot establish that the chip completed the requested conversion.
  Configure first, then acquire; the hysteresis setter that delegates to
  `configure()` has the same restriction. This is a non-breaking bug fix:
  preserving the in-flight busy indication across reconfiguration was
  never a documented or supported contract. Neither `configure()`'s
  signature nor `Error<E, P>` changed. `continuous()` does not route
  through `configure()` and its mode transitions are unaffected.
- Correct `one_shot()` documentation and doctest priming on both driver
  types (#60). The method remains a **bare trigger**, returning after the
  `M = 0b01` write; its signature and semantics are unchanged. Callers
  supervising acquisition themselves must first establish shutdown and
  allow deferred shutdown to settle (SBOS663A §7.4.1), then observe
  completion before reading temperature. The doctests now start from
  shutdown (`0x1020`, wire bytes `[0x20, 0x10]`), not the continuous-mode
  power-on word (`0x1022`, `[0x22, 0x10]`), and still expect the same
  trigger write (`0x1021`, `[0x21, 0x10]`). Use `acquire_one_shot` for the
  complete sequence, subject to its settling and interrupt-loss caveats.
- `AlertTmp108::wait_for_temperature_threshold` no longer loses an
  already-pending interrupt-mode alert (#59). Reading the configuration
  register clears both the watchdog flags and the ALERT pin (TMP108
  datasheet SBOS663A §7.5.3.4). The waiter now retains FL/FH from that
  entry read: if either flag was set, it reads the latest temperature
  without a GPIO wait or a second acknowledgment. Otherwise it waits for
  the asserted pin level, not an edge, also covering an assertion between
  the entry read and arming the GPIO wait. Comparator mode is unchanged.
  **Upgrade behavior:** a waiter that previously hung after a transient
  excursion may now return promptly, even with temperature back inside
  the configured band. A polling loop that appeared to work while the
  excursion persisted was receiving a later, relatched alert, not the
  original event. Calls no longer need that later event to complete.
  This is a behavioral bug fix; no public API or caller signature changes
  are required.
- Preserve an acknowledged interrupt's delivery obligation when the threshold
  waiter's temperature read fails or is cancelled (#58, Gap 2). The next call
  on the same wrapper reads temperature once, without configuration reads,
  GPIO waiting, or acknowledgment. Success clears the obligation; repeated
  failures retain it. Fresh comparator acquisition never retains delivery.
  **Upgrade behavior from 0.6.0:** a retry can now deliver an earlier interrupt
  instead of waiting for a new one, even after switching to Comparator mode.
  Direct sensor reads do not clear the obligation; decomposition or dropping
  the wrapper abandons it, and re-wrapping starts empty. Apply caller-side
  backoff or bounded retries: an immediately failing bus can make retries
  return errors without yielding. The driver does not retry internally.
  This is a **behaviour change to a documented contract, but not a
  source-breaking change**: no public API, signature, trait bound, or error
  variant changed.

### Documentation

- Correct the threshold waiter's returned-value description and alert
  examples (partial #58): the value is the most recent conversion, read
  after observing the alert, **not the temperature at time of trigger**.
  It may already be inside the configured band and cannot identify
  whether FL, FH, or both caused the event. The flags are not exposed to
  callers; #58 remains open for a status-bearing API. The earlier
  pending-edge guarantee described under 0.6.0 is superseded by the
  snapshot-and-level behavior above.
- Document that the threshold waiter is **not event-delivery cancel-safe**.
  Failure or cancellation during the entry or acknowledging configuration
  read can still consume an interrupt unrecoverably. Only the temperature
  stage after successful interrupt acknowledgment retains delivery for retry.
  `Error::Bus` does not identify the failed stage or prove no alert occurred.
  Retention is a delivery debt, not a cached or trigger-time sample: a retry
  reads the latest conversion, possibly inside the band and arbitrarily
  removed in time from the crossing. This is not exactly-once delivery;
  relatching or sustained comparator assertion can produce multiple `Ok`
  results for one physical excursion. Gap 1's status-bearing API remains open.

## [0.6.0] - 2026-09-04

The first release since 0.5.0, and a substantial one. It collects three
bodies of work: a reliability pass over the error and cleanup paths, the
split of the driver into separate blocking and async types, and the move
to **device-driver 2.1.0** with a typed temperature API.

Version numbers 0.6.0 and 0.7.0 were bumped in the repository during
development but never published to crates.io. Everything they contained
ships here, so this entry describes the whole delta from 0.5.0.

### Breaking

- The async driver is now `AsyncTmp108<I2C>`. Previously a single
  `Tmp108<I2C>` was generated by `maybe-async-cfg` and became async when
  the `async` feature was enabled, which meant enabling the feature
  *replaced* the blocking API rather than adding to it. The two types now
  coexist. **Migration:** in async code, replace `use tmp108::Tmp108;`
  with `use tmp108::AsyncTmp108;` and rename references accordingly.
- Temperatures crossing the driver boundary are `Celsius`, a newtype over
  sixteenths of a degree constrained to the part's 12-bit
  `-128.0..=127.9375 °C` range, instead of `f32`.
  - `temperature`, `low_limit`, `high_limit` and `wait_for_temperature`
    return `Celsius` on both `Tmp108` and `AsyncTmp108`.
    **Migration:** call `.to_degrees()` to recover the float.
  - `set_low_limit` and `set_high_limit` take `Celsius`.
    **Migration:** build the argument with `Celsius::try_from_degrees`.
  - The `embedded-sensors-hal` and `embedded-sensors-hal-async` trait
    impls are unaffected and still use `DegreesCelsius` (`f32`).
- `Error` is now `Error<E, P = core::convert::Infallible>` and gains a
  `Pin(P)` variant. The `Other` variant is removed.
  - Bare `Tmp108<I2C>` and `AsyncTmp108<I2C>` continue to use
    `Error<I2C::Error>`; the `Pin` variant is uninhabited
    (`P = Infallible`) and never produced.
  - `AlertTmp108<I2C, ALERT>` uses `Error<I2C::Error, ALERT::Error>` and
    surfaces GPIO failures via `Error::Pin(_)` instead of silently
    mapping them to `Other`.
- DDSL identifiers cannot start with `_`, so two enums were renamed:
  - `ConversionRate::_0_25Hz` → `ConversionRate::QuarterHz`
  - `ConversionRate::_1Hz` → `ConversionRate::OneHz`
  - `ConversionRate::_4Hz` → `ConversionRate::FourHz`
  - `ConversionRate::_16Hz` → `ConversionRate::SixteenHz`
  - `Hysteresis::_0C` → `Hysteresis::ZeroC`
  - `Hysteresis::_1C` → `Hysteresis::OneC`
  - `Hysteresis::_2C` → `Hysteresis::TwoC`
  - `Hysteresis::_4C` → `Hysteresis::FourC`
- `AlertTmp108::tmp108` is no longer a public field. **Migration:** use
  the `sensor()` / `sensor_mut()` accessors, or `into_inner()` to
  destructure into `(AsyncTmp108<I2C>, ALERT)`.
- `Tmp108::into_alert` is removed. The equivalent lives on
  `AsyncTmp108::into_alert`, where it is the correct flavor; the blocking
  driver never had a coherent path to the async-only `AlertTmp108`.
- `continuous` is async-only and lives on `AsyncTmp108`. It previously
  sat on `Tmp108` under `feature = "async"`, cfg-gated inside a
  maybe-async-cfg'd impl block.
- The `embedded-sensors-hal-async` feature now implicitly enables
  `async`. It was previously possible to build dead code by enabling the
  trait feature without `async`. Callers who listed both explicitly are
  unaffected.
- The MSRV is raised from 1.90.0 to 1.94.0, which device-driver 2.1.0
  requires.

### Behavior changes

- `continuous` now calls `shutdown()` on both the success and error paths
  of the user closure. Previously a closure failure short-circuited
  cleanup and left the chip in `Mode::Continuous` indefinitely. The
  closure's error takes precedence over a shutdown failure: the
  actionable signal wins. Callers that depended on the chip remaining in
  Continuous mode after a closure failure must call `configure(...)`
  explicitly.
- `set_temperature_threshold_hysteresis` snaps the input to the nearest
  legal value within a 0.05 °C tolerance band instead of requiring exact
  equality modulo `f32::EPSILON` (~1.2e-7). Inputs as ordinary as
  `0.1 + 0.9`, which previously failed, now succeed.
- Encoding a temperature rounds half away from zero where the old
  conversion truncated: 25.03 °C used to land on 25.0 °C and now lands on
  25.0625 °C.
- Decoding arithmetic-shifts the register right by four rather than
  scaling by 1/256, discarding the reserved low nibble instead of folding
  it into the result. The datasheet (SBOS663A) hardwires bits D3..D0 of
  the temperature register to 0 (Table 6, §7.5.2) and gives the same
  layout for T_LOW and T_HIGH (Table 12), so discarding them is correct.
  This also fixes the datasheet's own power-on T_HIGH value `0x7FF8`,
  which carries a stray bit 3: it used to decode as 127.96875 °C, outside
  the part's range, and now decodes as the documented 127.9375 °C.
- Out-of-range and non-finite limit values are rejected rather than
  silently saturated. In 0.5.0, `set_high_limit(f32::NAN)` cast through
  `as i16` and programmed a 0 °C trip point; such a value can no longer
  be constructed, because `Celsius::try_from_degrees` rejects it.

### Added

- `Celsius`, a newtype over sixteenths of a degree with the invariant
  `-2048..=2047`. Constructors `Celsius::from_sixteenths` and
  `Celsius::try_from_degrees`; accessors `sixteenths()` and
  `to_degrees()`. `Display` honours precision, so `{t:.2}` renders as
  before. Decoding a register is total: every one of the 65,536 possible
  bit patterns names a temperature in range.
- `OutOfRange`, the error returned by the `Celsius` constructors, with
  `TooLow`, `TooHigh` and `NotANumber` variants.
- `AsyncTmp108<I2C>`, parallel in surface to `Tmp108`. Both flavors
  expose `new`, `new_with_a0_gnd/vplus/sda/scl`, `addr`, `destroy`,
  `probe`, `read_configuration`, `configure`, `temperature`, `one_shot`,
  `shutdown`, `wait_for_temperature`, `low_limit`, `set_low_limit`,
  `high_limit` and `set_high_limit`. `AsyncTmp108` additionally provides
  `continuous` and `into_alert`.
- `probe()` reads the configuration register and reports whether it
  matches the documented power-on reset value (`0x1022`). Useful
  immediately after power-on. Returns `Ok(true)` for a POR match,
  `Ok(false)` if the chip is present but already configured, or `Err(_)`
  on bus failure.
- `AlertTmp108::sensor() -> &AsyncTmp108<I2C>` and
  `AlertTmp108::sensor_mut() -> &mut AsyncTmp108<I2C>`, replacing the
  former `pub tmp108` field.
- `AlertTmp108::into_inner(self) -> (AsyncTmp108<I2C>, ALERT)`, the
  inverse of `AsyncTmp108::into_alert`.
- `Eq` and `Hash` on `Config`.
- Conditional `Clone`, `Copy`, `PartialEq` and `Eq` on `Error<E, P>`,
  when `E` and `P` implement the corresponding trait.
- Exhaustive tests. The `Celsius` suite walks all 65,536 register words
  and all 4,096 representable temperatures rather than sampling, and a
  further suite walks the `Config` and `Mode` domains, including a
  two-sided assertion that the reserved `Mode` encoding is rejected and
  nothing else is.
- First-class doctest coverage for the async driver.

### Changed

- `device-driver` 1.0.9 → 2.1.0. The v2 register interface splits the
  `Error` and `AddressType` associated types out into a
  `RegisterInterfaceBase` supertrait, drops the `size_bits` parameter,
  adds a `&FieldsetMetadata` parameter, and makes writes take
  `&mut [u8]`. Both `Interface` and `AsyncInterface` are adapted.
  Fieldsets are emitted at the crate root instead of a `field_sets`
  submodule, and a fieldset's `Default` is now all-zeroes rather than the
  register reset value, which moved onto the register operation.
- The register description migrates from `tmp108.toml` to `tmp108.ddsl`;
  `src/inner.rs` is regenerated with device-driver-cli 2.1.0.
- With `default-features = false`, device-driver 2.1.0 pulls in no
  transitive dependencies at all. In v1 the `dsl`/`json`/`yaml`/`toml`
  features were on by default; in v2 they are opt-in and none is enabled.
- The driver no longer depends on `maybe-async-cfg`. The two driver types
  are written out directly and share their meaningful logic through a
  private `mod ops` of pure functions.
- The internal `Interface` is split into a blocking `Interface` (always
  available) and an async `AsyncInterface` (`#[cfg(feature = "async")]`).
- `ops::to_raw`, `ops::to_celsius`, `CELSIUS_PER_BIT`,
  `LIMIT_MIN_CELSIUS` and `LIMIT_MAX_CELSIUS` are superseded by `Celsius`
  and removed.
- The `cargo hack --feature-powerset` matrix shrinks from 8 to 6 legal
  combinations, since the two combos that disabled `async` while enabling
  `embedded-sensors-hal-async` are gone.
- Releases are automated with release-plz, publishing to crates.io via
  Trusted Publishing.
- Dependabot caps the number of open pull requests instead of ignoring
  patch and minor version bumps, which also suppressed security updates.

### Documentation

- `continuous` is documented as **not cancel-safe**: dropping the
  returned future, for example via `embassy_futures::select!` or
  `tokio::time::timeout`, leaves the chip in `Mode::Continuous`
  indefinitely.
- `wait_for_temperature` is documented as "wait one conversion period and
  then read". The first call after entering `Mode::Continuous` may return
  the previous conversion. Each call performs two I²C transactions; the
  documentation explains how to avoid the config-read overhead in
  bandwidth-sensitive loops.
- `AlertTmp108`'s threshold-wait behavior is documented in detail:
  Comparator-mode level-following (avoid tight loops; prefer Interrupt
  mode), the polarity-toggle race (do not reconfigure polarity during a
  pending wait), and reliance on the `embedded-hal-async` `Wait` trait
  contract for pending-edge handling.
- Crate-level documentation gains an "Operational notes" section covering
  the single-master I²C bus assumption and the driver-drop lifecycle.

### Migration

Most downstream code needs three changes.

Async users rename the type:

```rust
// Before
use tmp108::Tmp108;
let mut tmp = Tmp108::new_with_a0_gnd(i2c);

// After
use tmp108::AsyncTmp108;
let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
```

Temperatures are parsed and rendered rather than passed as floats:

```rust
// Before
let t: f32 = tmp.temperature()?;
tmp.set_high_limit(48.0)?;

// After
use tmp108::Celsius;
let t: f32 = tmp.temperature()?.to_degrees();
tmp.set_high_limit(Celsius::try_from_degrees(48.0).expect("48 C is representable"))?;
```

Code that pattern-matched on `Error::Other` switches to `Error::Pin(_)`
for the alert-pin case, and code that named `AlertTmp108`'s error type
explicitly changes `Error<I2C::Error>` to
`Error<I2C::Error, ALERT::Error>`.

## [0.5.0] and earlier

See git history (`git log --oneline v0.5.0`).
