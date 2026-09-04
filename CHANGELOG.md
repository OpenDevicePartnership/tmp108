# Changelog

All notable changes to this project are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
this project adheres to [Semantic Versioning](https://semver.org/).

## [0.8.0] — 2026-09-04

Two breaking changes land together: the register description moves to
**device-driver 2.1.0** and its DDSL manifest, and the temperature API
moves off `f32` onto a **`Celsius`** newtype.

### Breaking

- Temperatures crossing the driver boundary are now `Celsius`, a
  newtype over sixteenths of a degree constrained to the part's 12-bit
  `-128.0..=127.9375 °C` range, instead of `f32`.
  - `temperature`, `low_limit`, `high_limit` and
    `wait_for_temperature` return `Celsius` on both `Tmp108` and
    `AsyncTmp108`. **Migration:** call `.to_degrees()` to recover the
    float.
  - `set_low_limit` and `set_high_limit` take `Celsius`.
    **Migration:** build the argument with `Celsius::try_from_degrees`.
- `set_low_limit` and `set_high_limit` return
  `Result<(), I2C::Error>` again, not `Result<(), Error<I2C::Error>>`.
  Their only non-bus failure was `Error::InvalidInput`, raised when the
  `f32` argument was unrepresentable; a `Celsius` cannot be
  unrepresentable, so the variant became unreachable. `Error::InvalidInput`
  itself stays — the `embedded-sensors-hal` and
  `embedded-sensors-hal-async` trait impls keep their
  `DegreesCelsius` (`f32`) signatures and still need it.
- DDSL identifiers cannot start with `_`, so two enums were renamed:
  - `ConversionRate::_0_25Hz` → `ConversionRate::QuarterHz`
  - `ConversionRate::_1Hz` → `ConversionRate::OneHz`
  - `ConversionRate::_4Hz` → `ConversionRate::FourHz`
  - `ConversionRate::_16Hz` → `ConversionRate::SixteenHz`
  - `Hysteresis::_0C` → `Hysteresis::ZeroC`
  - `Hysteresis::_1C` → `Hysteresis::OneC`
  - `Hysteresis::_2C` → `Hysteresis::TwoC`
  - `Hysteresis::_4C` → `Hysteresis::FourC`
- The MSRV is raised from 1.90.0 to 1.94.0, which device-driver 2.1.0
  requires.

### Behavior changes

- Encoding a temperature now rounds half away from zero where the old
  `ops::to_raw` truncated: 25.03 °C used to land on 25.0 °C and now
  lands on 25.0625 °C.
- Decoding now arithmetic-shifts the register right by four rather than
  scaling by 1/256, discarding the reserved low nibble instead of
  folding it into the result. The datasheet (SBOS663A) hardwires bits
  D3..D0 of the temperature register to 0 (Table 6, §7.5.2) and gives
  the same layout for T_LOW and T_HIGH (Table 12), so discarding them
  is correct. This also fixes the datasheet's own power-on T_HIGH value
  `0x7FF8`, which carries a stray bit 3: it used to decode as
  127.96875 °C, outside the part's range, and now decodes as the
  documented 127.9375 °C.

### Added

- `Celsius`, a newtype over sixteenths of a degree with the invariant
  `-2048..=2047`. Constructors `Celsius::from_sixteenths` and
  `Celsius::try_from_degrees`; accessors `sixteenths()` and
  `to_degrees()`. `Display` honours precision, so `{t:.2}` renders as
  before. Decoding a register is total: every one of the 65,536
  possible bit patterns names a temperature in range.
- `OutOfRange`, the error returned by the `Celsius` constructors, with
  `TooLow`, `TooHigh` and `NotANumber` variants.
- Exhaustive tests: the `Celsius` suite walks all 65,536 register words
  and all 4,096 representable temperatures rather than sampling, and a
  new suite walks the `Config` and `Mode` domains exhaustively.

### Changed

- `device-driver` 1.0.9 → 2.1.0. The v2 register interface splits the
  `Error` / `AddressType` associated types out into a
  `RegisterInterfaceBase` supertrait, drops the `size_bits` parameter,
  adds a `&FieldsetMetadata` parameter, and makes writes take
  `&mut [u8]`; both `Interface` and `AsyncInterface` are adapted.
  Fieldsets are emitted at the crate root instead of a `field_sets`
  submodule, and a fieldset's `Default` is now all-zeroes rather than
  the register reset value (which moved onto the register operation).
- The register description migrates from `tmp108.toml` to
  `tmp108.ddsl`; `src/inner.rs` is regenerated with
  device-driver-cli 2.1.0.
- With `default-features = false`, device-driver 2.1.0 pulls in no
  transitive dependencies at all — v1's default `dsl`/`json`/`yaml`/
  `toml` features became opt-in and none are enabled.
- `ops::to_raw`, `ops::to_celsius`, `CELSIUS_PER_BIT`,
  `LIMIT_MIN_CELSIUS` and `LIMIT_MAX_CELSIUS` are superseded by
  `Celsius` and removed.
- Releases are automated with release-plz.
- Dependabot now caps the number of open PRs instead of ignoring
  version bumps.

## [0.7.0] — 2026-06-04

An architectural refactor: the single `maybe-async-cfg`-generated
`Tmp108<I2C>` is replaced by two named driver types, **`Tmp108`** for
the blocking flavor and **`AsyncTmp108`** for the async flavor. Both
are now available simultaneously when both relevant features are
enabled. The shared register codec lives in a new private `mod ops`.

### Breaking

- The async driver type is now `AsyncTmp108<I2C>` (was `Tmp108<I2C>`
  under `feature = "async"`). The blocking `Tmp108<I2C>` is
  unchanged. **Migration:** in async code, replace `use tmp108::Tmp108;`
  with `use tmp108::AsyncTmp108;` and rename references accordingly.
- `AlertTmp108::tmp108` is no longer a public field. **Migration:**
  use the new `sensor()` / `sensor_mut()` accessors, or `into_inner()`
  to destructure into `(AsyncTmp108<I2C>, ALERT)`.
- The `embedded-sensors-hal-async` feature now implicitly enables
  `async` (it was previously possible to build dead code by enabling
  the trait feature without `async`). Callers who listed both
  explicitly are unaffected.
- `Tmp108::into_alert` is removed; the equivalent now lives only on
  `AsyncTmp108::into_alert`, where it is the correct flavor (the
  bare blocking driver never had a coherent path to the async-only
  `AlertTmp108` wrapper).
- `AsyncTmp108::continuous` is async-only and lives only on the
  async driver type (it previously lived on `Tmp108` under
  `feature = "async"` and was awkwardly cfg-gated within a
  maybe-async-cfg'd impl block).

### Added

- `AsyncTmp108<I2C>` driver type, parallel in surface to `Tmp108`.
  Both flavors expose: `new`, `new_with_a0_gnd/vplus/sda/scl`,
  `addr`, `destroy`, `probe`, `read_configuration`, `configure`,
  `temperature`, `one_shot`, `shutdown`, `wait_for_temperature`,
  `low_limit`, `set_low_limit`, `high_limit`, `set_high_limit`.
  `AsyncTmp108` additionally provides `continuous` and
  `into_alert`.
- `AlertTmp108::sensor() -> &AsyncTmp108<I2C>` and
  `AlertTmp108::sensor_mut() -> &mut AsyncTmp108<I2C>` replace the
  former `pub tmp108` field.
- `AlertTmp108::into_inner(self) -> (AsyncTmp108<I2C>, ALERT)` —
  inverse of `AsyncTmp108::into_alert`.
- `Eq` and `Hash` on `Config`.
- Conditional `Clone`, `Copy`, `PartialEq`, `Eq` on `Error<E, P>`
  (when `E` and `P` implement the corresponding trait).
- First-class doctest coverage for the async driver. Doctest count
  with `--all-features` rises from 18 to 47.

### Internal

- The driver no longer depends on `maybe-async-cfg`. The two driver
  types are written out directly and share their meaningful logic
  through a private `mod ops` of pure functions
  (`to_celsius`, `to_raw`, `snap_hysteresis`, `decode_config`,
  `apply_config`, plus the relevant constants).
- The internal `Interface` is split into a blocking `Interface`
  (always available) and an async `AsyncInterface`
  (`#[cfg(feature = "async")]`). Each implements the matching
  `device-driver` register-interface trait.
- The `# fn main()` doctest workaround (formerly Gotcha 3 in
  AGENTS.md) is gone — each type has its own doctests with the
  correct shape for its flavor.
- `cargo hack --feature-powerset` matrix shrinks from 8 to 6 legal
  combinations (the two combos that disabled `async` while enabling
  `embedded-sensors-hal-async` are gone).

## [0.6.0] — 2026-06-03

A reliability-focused release. Several `Error` paths and `Tmp108`
method signatures change in ways that are technically breaking but
trivially mechanical to migrate; see the **Migration** section below.

### Breaking

- `Error` is now `Error<E, P = core::convert::Infallible>` and gains
  a [`Pin(P)`] variant. The [`Other`] variant is removed.
  - Bare `Tmp108<I2C>` continues to use `Error<I2C::Error>`; the
    `Pin` variant is uninhabited (`P = Infallible`) and never
    produced.
  - `AlertTmp108<I2C, ALERT>` now uses
    `Error<I2C::Error, ALERT::Error>` and surfaces GPIO failures via
    `Error::Pin(_)` instead of silently mapping them to `Other`.
- `Tmp108::set_low_limit` and `Tmp108::set_high_limit` now return
  `Result<(), Error<I2C::Error>>` instead of `Result<(), I2C::Error>`.
  They reject NaN, ±∞, and anything outside the chip's representable
  range `[-128.0, 127.9375] °C` with `Error::InvalidInput`.
  Previously these inputs were silently saturated via `as i16`.

### Behavior changes (non-breaking signatures)

- `Tmp108::continuous` now calls `shutdown()` on both the success and
  error paths of the user closure. Previously a closure failure would
  short-circuit cleanup and leave the chip in `Mode::Continuous`
  indefinitely. The closure's error takes precedence over a shutdown
  failure: the actionable signal wins. Callers that depended on the
  chip remaining in Continuous mode after a closure failure must call
  `configure(...)` explicitly.
- `set_temperature_threshold_hysteresis` (on both `Tmp108` and
  `AlertTmp108`) now snaps the input to the nearest legal value
  within a 0.05 °C tolerance band instead of requiring exact equality
  modulo `f32::EPSILON` (~1.2e-7). Inputs as ordinary as `0.1 + 0.9`
  that previously failed will now succeed.
- `to_celsius` no longer uses asymmetric integer division before the
  float conversion. The new computation
  (`f32::from(t) * (CELSIUS_PER_BIT / 16.0)`) produces identical
  results for datasheet-conforming inputs (bits 3..0 == 0) and the
  correct symmetric result for any non-zero low bits.

### Added

- `Tmp108::probe()` reads the configuration register and reports
  whether it matches the documented power-on reset value (`0x1022`).
  Useful immediately after power-on. Returns `Ok(true)` for POR
  match, `Ok(false)` if the chip is present but already configured,
  or `Err(_)` on bus failure.

### Documentation

- `Tmp108::continuous` is documented as **not cancel-safe**: dropping
  the returned future (e.g. via `embassy_futures::select!` or
  `tokio::time::timeout`) leaves the chip in `Mode::Continuous`
  indefinitely.
- `Tmp108::wait_for_temperature` is documented as "wait one
  conversion period and then read." The first call after entering
  `Mode::Continuous` may return the previous conversion. Each call
  performs two I²C transactions; the doc explains how to avoid the
  config-read overhead in bandwidth-sensitive loops.
- `AlertTmp108`'s threshold-wait behavior is documented in detail:
  Comparator-mode level-following (avoid tight loops; prefer
  Interrupt mode), polarity-toggle race (do not reconfigure polarity
  during a pending wait), and reliance on the `embedded-hal-async`
  `Wait` trait contract for pending-edge handling.
- Crate-level docs gain an "Operational notes" section covering the
  single-master I²C bus assumption and the driver-drop lifecycle.

### Migration

For most downstream code the only required change is a single error
pattern:

```rust
// Before
match tmp.set_low_limit(value) {
    Ok(()) => {}
    Err(i2c_err) => /* handle */,
}

// After
match tmp.set_low_limit(value) {
    Ok(()) => {}
    Err(Error::Bus(i2c_err)) => /* handle bus error */,
    Err(Error::InvalidInput) => /* handle invalid f32 (new) */,
}
```

Downstream code that pattern-matched on `Error::Other` should switch
to `Error::Pin(_)` for the alert-pin case. Code that named
`AlertTmp108`'s `Error` type explicitly should change
`Error<I2C::Error>` to `Error<I2C::Error, ALERT::Error>`.

## [0.5.0] and earlier

See git history (`git log --oneline v0.5.0`).
