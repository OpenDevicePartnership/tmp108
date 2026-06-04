# Changelog

All notable changes to this project are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
this project adheres to [Semantic Versioning](https://semver.org/).

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
