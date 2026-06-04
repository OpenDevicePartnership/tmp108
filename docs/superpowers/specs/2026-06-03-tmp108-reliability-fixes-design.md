# TMP108 Reliability Fixes — Design Spec

**Status:** approved (interactive Q&A 2026-06-03)
**Target release:** 0.6.0 (breaking)
**Branch:** `reliability-fixes`
**Companion plan:** `docs/superpowers/plans/2026-06-03-tmp108-reliability-fixes.md`

---

## Problem statement

A reliability review of `tmp108` 0.5.0 surfaced a set of operational
hazards in `src/lib.rs`. This document captures the agreed scope, the
chosen fix for each item, and the resulting public-API impact. A
companion architectural review proposed dropping `maybe-async-cfg` in
favor of two named types (`Tmp108` / `AsyncTmp108`); **that work is
explicitly deferred** to a follow-up release. This release does
reliability fixes only, but is allowed to break API (and bump to 0.6.0)
where doing so produces a more honest signature.

## Goals

1. Eliminate or document every behavior that can silently leave the
   chip in an unsafe state in the field.
2. Eliminate or document every behavior that loses caller-visible
   error information.
3. Land all of (1) and (2) in a single, bisectable PR with one commit
   per fix.
4. Bump to 0.6.0 with `BREAKING CHANGE:` footers where applicable.

## Non-goals

- Architectural restructuring (typestate, separate types, removing
  `maybe-async-cfg`) — deferred.
- Hardware verification of TLow/THigh resolution (H3) — requires lab
  time and is orthogonal to the code changes.
- Probe-on-construct (`new()` becoming fallible) — explicitly rejected
  in review.
- Diagnostic counters (L5) — out of scope.
- I²C clock-stretching exposure (L4) — not a driver issue.

---

## Fix decisions (Critical → Medium)

### C1. `continuous()` cancel-safety

**Status:** doc-only.

`Tmp108::continuous` writes `Mode::Continuous`, runs the user closure,
then writes `Mode::Shutdown`. If the returned future is dropped (e.g.
by `embassy_futures::select!`, `tokio::time::timeout`, or a task
cancellation) the cleanup never runs and the chip is left burning
current.

A true RAII fix requires synchronous I²C inside an `async Drop`, which
stable Rust does not support and which embedded HALs do not promise
either. Solution: document loudly on the method, on the crate, and in
`AGENTS.md` that **the returned future is not cancel-safe**. Callers
must own the future to completion or arrange their own recovery.

No new API. No signature change.

### C2. `continuous()` error path

**Status:** behavior change, non-breaking.

Today: closure `Err` short-circuits via `?`, skipping `shutdown()`.

After: closure result is captured, `shutdown()` runs unconditionally,
**closure error wins** over shutdown error (matches user intent — the
caller's failure is the actionable signal; shutdown failure is a
secondary cleanup symptom).

```rust
let user = f(self).await;
let cleanup = self.shutdown().await;
user.and(cleanup)   // Ok only if both succeeded; closure error preferred over shutdown error
```

`.and` returns the first `Err`, or the second `Ok` — correct for "user
error wins, cleanup error reported if user succeeded."

No signature change. Behavior change is observable (callers that
relied on chip being in Continuous after a closure failure will
notice). Documented in CHANGELOG.

### C3. Range validation on `set_low_limit` / `set_high_limit`

**Status:** breaking — return type changes.

Today: `pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error>`
silently saturates inputs outside `[-128.0, 127.9375]` via `as i16`.

After: reject NaN, ±Inf, and anything outside `[-128.0, 127.9375]`
with `Error::InvalidInput`.

Signature changes to:

```rust
pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), Error<I2C::Error>>
pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), Error<I2C::Error>>
```

**BREAKING:** the success path is unchanged but the error type is
now `Error<I2C::Error>` not `I2C::Error`. Callers that pattern-match
on `I2C::Error` directly need to extract via `Error::Bus(e) => ...`.

The validation range `[-128.0, 127.9375]` is the full representable
range of the 12-bit fixed-point register, not the TMP108's
operating-temperature window (−55…+127 °C). The wider range is
chosen because the chip itself accepts these values; the operating
window is a board-design concern, not a driver invariant.

### H1. AlertTmp108 Interrupt-mode lost edges

**Status:** rejected as a "fix"; documented as a contract.

The reliability review proposed sampling `InputPin::is_low`/`is_high`
on entry. User clarification: **the `Wait` trait already handles
pending edges via the MCU's GPIO controller** — that's the contract
of `embedded_hal_async::digital::Wait`. Implementations that drop
pending edges between calls are non-conforming; that is the HAL
implementor's responsibility, not the driver's.

Action: add a brief doc note on `wait_for_temperature_threshold`
referencing the `Wait` contract. No code change.

### H2. AlertTmp108 Comparator-mode busy-loop

**Status:** documented.

`wait_for_low`/`wait_for_high` return immediately when the line is
already at the requested level. In Comparator mode the ALERT pin
stays asserted until the temperature returns to the hysteresis band.
Callers writing a `loop { wait...; }` pattern will spin.

Action: document on the method that Comparator-mode callers should
either (a) use Interrupt mode for level-following semantics, or (b)
apply application-level backoff between iterations. No code change
(the `Wait` trait does not expose a "wait for deassertion" primitive
that would let us do this cleanly without an opinionated semantic
change).

### H3. TLow/THigh resolution verification

**Status:** out of scope.

Hardware-only verification. Tracked as a TODO comment in the
hardware-verification matrix.

### H4. Stale-first-reading in `wait_for_temperature`

**Status:** documented.

The delay table is the conversion *period* (1/CR), not the conversion
*time*. First call after entering Continuous can return a stale
value. Documentation on the method explains the semantics: "wait one
period and then read; the first read after entering Continuous may
return the previous conversion. For 'guaranteed fresh,' use
`one_shot()` + period delay + `temperature()`."

No code change.

### H5. `Hysteresis` snapping

**Status:** behavior change, non-breaking.

Today: `(hysteresis - 1.0).abs() < f32::EPSILON` is too strict
(1.2e-7 tolerance). `0.1 + 0.9` fails.

After: snap to nearest of `{0, 1, 2, 4}` within a 0.05 °C tolerance
band; beyond tolerance return `Error::InvalidInput`.

```rust
// Pseudo-code; final form in the implementation step.
const HYS_VALUES: [(f32, Hysteresis); 4] = [
    (0.0, Hysteresis::_0C),
    (1.0, Hysteresis::_1C),
    (2.0, Hysteresis::_2C),
    (4.0, Hysteresis::_4C),
];
let (legal, h) = HYS_VALUES
    .iter()
    .copied()
    .min_by(|(a, _), (b, _)| (hysteresis - a).abs().total_cmp(&(hysteresis - b).abs()))
    .unwrap();
if (hysteresis - legal).abs() > 0.05 {
    return Err(Error::InvalidInput);
}
```

(NaN handling: `total_cmp` orders NaN to one end; will likely fall
outside any 0.05 tolerance and reject. Explicit NaN check added to
be safe.)

Same signature, same return type — callers that previously rejected
`0.1 + 0.9` will now accept it. Non-breaking improvement.

### H6. `Error::Other` swallows GPIO error

**Status:** breaking — `Error` gains a type parameter.

Today:

```rust
pub enum Error<E: embedded_hal::i2c::Error> {
    Bus(E),
    InvalidInput,
    Other,
}
```

After:

```rust
pub enum Error<E: embedded_hal::i2c::Error, P: embedded_hal::digital::Error = core::convert::Infallible> {
    Bus(E),
    InvalidInput,
    Pin(P),
}
```

(`Other` is removed — every existing producer of `Error::Other` is
either a GPIO error that now uses `Error::Pin(...)` or a now-corrected
swallowed `Bus(...)`.)

`AlertTmp108<I2C, ALERT>` produces `Error<I2C::Error, ALERT::Error>`.
Bare `Tmp108<I2C>` continues to use the default `P =
core::convert::Infallible` (it never reports a Pin error).

**BREAKING:**

- The `P` type parameter is new. Pattern matches against `Error::Other`
  must be replaced (recommended migration: pattern on `Error::Pin(e)`
  for GPIO failures, keep `Bus`/`InvalidInput` matches as-is).
- The `Default` for `P` keeps `Error<I2C::Error>` working at most
  bare-`Tmp108` call sites.

### M1. RMW atomicity on multi-master

**Status:** documented.

Add a "Concurrency" section to the crate-level doc explaining the
single-master assumption.

### M2. `probe()` method

**Status:** new non-breaking method.

```rust
/// Probe the bus by reading the configuration register and confirming the
/// POR value (`0x1022`). Useful immediately after power-on. False negatives
/// possible if the chip was already reconfigured.
pub async fn probe(&mut self) -> Result<(), I2C::Error>
```

Returns `Ok(())` if the read succeeds and matches POR. Returns the
`I2C::Error` if the read fails. Returns `Ok(())` *only* if it matches
POR — if the read succeeds but the value differs (chip was already
configured), returns `Ok(())` as well? **No** — we surface this as a
separate signal: the method is honest about "did I see POR?" by
returning `Ok(true)` for POR match, `Ok(false)` for read-succeeded-
but-not-POR, and `Err(...)` for read failure.

Final shape:

```rust
pub async fn probe(&mut self) -> Result<bool, I2C::Error>
```

Documentation: "Returns `Ok(true)` if the configuration register reads
the documented POR value `0x1022` (chip is freshly powered up and
present), `Ok(false)` if the read succeeded but the chip was already
configured to a different state (still strong evidence the chip is
present), `Err(...)` if the read failed (chip likely missing or
unreachable)."

### M3. `Drop` lifecycle clarity

**Status:** documented.

Add a `# Lifecycle` section to the `Tmp108` doc comment explaining
that dropping the driver does not change the chip state, and that
callers should `shutdown()` explicitly before dropping if they want
the chip to stop consuming idle current.

### M4. Polarity-toggle race

**Status:** documented.

Doc note on `wait_for_temperature_threshold` that polarity must not
be changed concurrently with a pending call.

### M5. `to_celsius` integer division

**Status:** code change, non-breaking.

Replace:

```rust
fn to_celsius(t: i16) -> f32 {
    f32::from(t / 16) * Self::CELSIUS_PER_BIT
}
```

with:

```rust
fn to_celsius(t: i16) -> f32 {
    f32::from(t) * (Self::CELSIUS_PER_BIT / 16.0)
}
```

`CELSIUS_PER_BIT / 16.0 = 0.00390625`, exactly representable. Same
results on legal datasheet-conforming inputs (low 4 bits = 0).
Asymmetric for negative inputs with non-zero low bits — fixed.
Existing unit tests at `src/lib.rs:1300-1421` (blocking) and
`src/lib.rs:1497-1618` (async) cover the change.

### M6. Per-sample config read in `wait_for_temperature`

**Status:** documented.

Doc note on `wait_for_temperature` that each call performs an extra
I²C round trip to read the configuration register. Callers that
need to minimize bus traffic should call `read_configuration()`
once and implement the period delay themselves.

---

## Public-API impact summary

### Breaking changes

| Symbol | Before | After |
|---|---|---|
| `Error<E>` | `Error<E: embedded_hal::i2c::Error>` | `Error<E, P = core::convert::Infallible>` |
| `Error::Other` | variant exists | removed |
| `Tmp108::set_low_limit` | `-> Result<(), I2C::Error>` | `-> Result<(), Error<I2C::Error>>` |
| `Tmp108::set_high_limit` | `-> Result<(), I2C::Error>` | `-> Result<(), Error<I2C::Error>>` |
| `AlertTmp108` trait `Error` | `Error<I2C::Error>` (with `P = Infallible` impossible because pin error was mapped to `Other`) | `Error<I2C::Error, ALERT::Error>` |

### Non-breaking additions

| Symbol | Purpose |
|---|---|
| `Tmp108::probe` | confirm chip presence + POR |
| `Error::Pin(P)` variant | preserve GPIO error |

### Behavior changes (same signature)

| Method | Change |
|---|---|
| `Tmp108::continuous` | shutdown now runs on closure `Err`; closure error wins |
| `set_temperature_threshold_hysteresis` | tolerance widened from `f32::EPSILON` to 0.05 °C, snap to nearest |
| internal `to_celsius` | integer division removed |

### Documentation-only changes

- `continuous()` — cancel-safety contract
- `wait_for_temperature_threshold` — `Wait` contract reference, polarity race note
- `wait_for_temperature` — stale-first-reading note, per-call I²C cost note
- crate-level — single-master assumption, lifecycle (drop) note

---

## Migration notes (for CHANGELOG.md)

```
## 0.6.0

### Breaking

- `Error` is now `Error<E, P = core::convert::Infallible>` and gains a
  `Pin(P)` variant. The `Other` variant is removed. `AlertTmp108`
  errors now carry the underlying GPIO error in `Pin(P)`; previously
  they were silently mapped to `Other`.
- `Tmp108::set_low_limit` and `set_high_limit` now return
  `Result<(), Error<I2C::Error>>` and reject NaN / ±Inf / out-of-range
  inputs with `Error::InvalidInput`. Previously they silently saturated.
- `Tmp108::continuous` now calls `shutdown()` on both the success and
  error paths of the user closure. Callers that depended on the chip
  remaining in Continuous mode after a closure failure must call
  `configure(...)` explicitly.

### Added

- `Tmp108::probe()` confirms chip presence and reports whether the
  configuration register matches the documented POR value.

### Fixed

- `set_temperature_threshold_hysteresis` no longer rejects float
  arithmetic that lands within 0.05 °C of a legal value
  (previously the tolerance was `f32::EPSILON`).
- `to_celsius` no longer uses asymmetric integer division before the
  float conversion.

### Documented

- `continuous()` future is not cancel-safe.
- `wait_for_temperature_threshold` polarity must not change concurrently.
- `wait_for_temperature` may return stale data on the first call after
  entering Continuous mode.
- The driver assumes single-master I²C bus ownership.
- Dropping the driver does not change the chip's operating mode.
```

---

## Commit plan (bisectable, one concern per commit)

Order chosen so each commit is independently buildable and tests at
its level of coverage pass. Per AGENTS.md Gotcha 6 ("each commit
builds clean without warning") each commit contains both the new
tests and the implementation that satisfies them — TDD is preserved
in the workflow (write test, watch it fail, implement) but the
failing-test state is never committed.

1. `feat!: validate set_*_limit inputs, return Error::InvalidInput` (C3) **BREAKING**
2. `fix: run shutdown() in continuous() on both success and error paths` (C2)
3. `fix: snap hysteresis input to nearest legal value within 0.05C` (H5)
4. `refactor: remove integer division from to_celsius` (M5)
5. `feat!: replace Error::Other with Error::Pin generic over ALERT::Error` (H6) **BREAKING**
6. `feat: add Tmp108::probe() method` (M2)
7. `docs: cancel-safety, lifecycle, concurrency, stale-read warnings` (C1, H1, H2, H4, M1, M3, M4, M6)
8. `chore: bump version to 0.6.0 and add CHANGELOG`

Each commit:
- Builds clean under all four feature combinations (`cargo hack`).
- Passes `cargo clippy --all-features --all-targets` with the
  pedantic config.
- Passes `cargo +nightly fmt --check`.
- Passes `cargo test` at its applicable feature levels.
- Has the `Assisted-by:` trailer per AGENTS.md.

The version bump and CHANGELOG are intentionally last so that
`cargo semver-checks` flags each breaking commit individually during
bisect.

---

## Out of scope for this PR (tracked for follow-up)

- Architectural refactor to two named types `Tmp108` / `AsyncTmp108`
  (architect's recommendation; will be a separate plan after this PR
  is integrated).
- Hardware verification of TLow/THigh resolution (H3).
- Diagnostic counters (L5).
- `Eq` on `Config` (low-priority cleanup, can wait for the
  architectural PR).
- Making `AlertTmp108::tmp108` private + adding `into_inner()` (will
  be addressed by the named-types refactor anyway).
- `Clone`/`PartialEq` on `Error` (depends on `E: Clone + PartialEq`
  which is uncommon; skip).

---

## Risks

1. **The H6 breaking change has wide blast radius.** Every match on
   `Error::Other` in downstream code breaks. The `Pin` variant is a
   real semantic improvement but the migration is mechanical. CHANGELOG
   covers it.
2. **Adding a generic parameter with a default to a public enum** is
   *binary* breaking (size/layout can shift) but *source* compatible
   for typical use because of `P = Infallible`. `cargo semver-checks`
   will flag it as major; the explicit 0.6.0 bump is the right answer.
3. **The C2 behavior change (shutdown on error) may surprise** users
   who deliberately wanted to inspect the chip after a closure failure.
   The CHANGELOG is explicit; the new behavior matches the safer
   default and matches what most callers expect.
4. **Per-commit clippy/test stability.** TDD is followed in the
   working tree (write the test, watch it fail, implement) but the
   failing-test state is never committed; each commit lands test +
   implementation together. This satisfies AGENTS.md Gotcha 6 ("each
   commit builds clean without warning") while preserving the
   discipline.
