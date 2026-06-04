# TMP108 Type-Split Refactor — Design Spec

**Status:** approved (interactive Q&A 2026-06-04)
**Target release:** 0.7.0 (breaking)
**Branch:** `named-types` (based on `reliability-fixes`)
**Predecessor PR:** `reliability-fixes` (0.6.0)
**Companion plan:** none — see this spec's "Commit plan" section.

---

## Problem statement

The driver currently emits both blocking and async variants of
`Tmp108<I2C>` from a single source via `maybe-async-cfg`. The
architect's review found that:

- The "single source" promise is already broken — every public method
  body contains hand-written `#[cfg(feature = "async")]` arms that
  duplicate the meaningful work.
- The trait impls (`embedded-sensors-hal*`, `AlertTmp108`) are written
  by hand twice.
- The test module is duplicated as `tests::blocking` and
  `tests::asynchronous`.
- Doctests cannot satisfy both blocking and async signatures from one
  body, forcing the `# #[cfg(feature = "async")] fn main() {}` shim
  (Gotcha 3) on every doctest.
- Async paths have **zero** doctest coverage today; only the blocking
  variant runs against doctests.

The architect's recommendation (Option B + refinement G): replace the
macro with two distinct, named types (`Tmp108` blocking,
`AsyncTmp108` async), share register encoding/decoding via a private
`mod ops` of pure functions, and drop `maybe-async-cfg`.

## Goals

1. Two named driver types: `Tmp108<I2C>` (blocking, `embedded-hal`
   1.0) and `AsyncTmp108<I2C>` (async, `embedded-hal-async` 1.0).
2. Both types available simultaneously when both features are
   enabled.
3. Drop the `maybe-async-cfg` build-time dependency.
4. Drop the `# fn main()` doctest shim (Gotcha 3 in AGENTS.md goes
   away).
5. Async paths get real doctest coverage for the first time.
6. Pure-function register encoding/decoding in a private `mod ops`
   (refinement G).
7. Architect's listed bonus cleanups: `AlertTmp108` field privacy +
   round-trip, `Eq` on `Config`, derive cleanups on `Error`.
8. Bisectable history, one concern per commit, AGENTS.md "each commit
   builds clean" rule respected.

## Non-goals

- Adding new functionality beyond the existing API surface (no new
  methods, no new public types beyond the two driver types and their
  wrappers).
- Changing the chip-facing wire protocol (the wire-level mock
  expectations in existing tests must continue to pass unmodified
  modulo the type-name change).
- Changing `AlertTmp108`'s blocking-vs-async story — it stays
  async-only and wraps `AsyncTmp108`.
- Touching the reliability fixes themselves (all PR1 behavior is
  preserved verbatim).

---

## Design decisions

### D1. Type naming

- Blocking: `pub struct Tmp108<I2C>` (unchanged name from current
  blocking-feature build).
- Async: `pub struct AsyncTmp108<I2C>` (new name; was also named
  `Tmp108` under `feature = "async"` previously).
- `AlertTmp108<I2C, ALERT>`: stays async-only, holds an
  `AsyncTmp108<I2C>`.

Rationale: matches the `embedded-storage` / `embedded-storage-async`
naming convention. Downstream users who currently call the bare
driver `Tmp108` keep working in the blocking case; async users do a
single mechanical `Tmp108 → AsyncTmp108` migration.

### D2. Feature gating

`Cargo.toml`:

```toml
[features]
async = ["dep:embedded-hal-async"]
embedded-sensors-hal = ["dep:embedded-sensors-hal"]
embedded-sensors-hal-async = ["dep:embedded-sensors-hal-async", "async"]
```

Note the explicit dependency: `embedded-sensors-hal-async` now
*requires* `async` rather than silently producing dead code when used
without it. This was an inconsistency in 0.6.0 that this PR fixes.

What lives where:

- `pub struct Tmp108<I2C>` — always available.
- `pub struct AsyncTmp108<I2C>` — `#[cfg(feature = "async")]`.
- `pub struct AlertTmp108<I2C, ALERT>` —
  `#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]`
  (unchanged from 0.6.0; the `async` requirement is now also enforced
  at the feature-level).
- Internal `Interface` (blocking) — always available.
- Internal `AsyncInterface` — `#[cfg(feature = "async")]`.

### D3. Shared `mod ops` (refinement G)

Private module containing pure functions with no I/O:

```rust
mod ops {
    use crate::{Config, Hysteresis};
    use crate::inner::{ConversionRate, Mode, Polarity, Thermostat};

    pub const CELSIUS_PER_BIT: f32 = 0.0625;

    /// Convert a raw 16-bit signed register value to °C.
    pub fn to_celsius(t: i16) -> f32 { /* ... */ }

    /// Convert °C to a raw 12-bit signed left-aligned register value.
    /// Returns None for NaN, ±∞, or out-of-range inputs.
    pub fn to_raw(t: f32) -> Option<i16> { /* ... */ }

    /// The chip's documented power-on-reset configuration register
    /// value (0x1022 LE).
    pub const POR_CONFIG: u16 = 0x1022;

    /// Decode a configuration-register snapshot into a typed Config.
    pub fn decode_config(c: ConfigurationFieldSet) -> Config { /* ... */ }

    /// Apply a typed Config to a configuration-register snapshot,
    /// preserving untouched bits (M, FL, FH, ID).
    pub fn apply_config(r: &mut ConfigurationFieldSet, cfg: &Config) { /* ... */ }

    /// Snap a continuous-f32 hysteresis input to the nearest legal
    /// chip setting within the 0.05 °C tolerance band, or return None
    /// if out of band / non-finite.
    pub fn snap_hysteresis(input: f32) -> Option<Hysteresis> { /* ... */ }
}
```

`Tmp108::temperature` then becomes:

```rust
pub fn temperature(&mut self) -> Result<f32, I2C::Error> {
    let raw = self.inner.temperature().read()?;
    Ok(ops::to_celsius(i16::from_be_bytes(raw.into())))
}
```

and the async equivalent:

```rust
pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
    let raw = self.inner.temperature().read_async().await?;
    Ok(ops::to_celsius(i16::from_be_bytes(raw.into())))
}
```

The two shells are obviously parallel; the meaningful logic
(`to_celsius`, range validation, hysteresis snapping, config
decode/encode) lives once.

### D4. `Interface` / `AsyncInterface`

Two parallel internal structs:

```rust
struct Interface<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
}
impl<I2C: embedded_hal::i2c::I2c> device_driver::RegisterInterface for Interface<I2C> { ... }

#[cfg(feature = "async")]
struct AsyncInterface<I2C: embedded_hal_async::i2c::I2c> {
    i2c: I2C,
    addr: u8,
}
#[cfg(feature = "async")]
impl<I2C: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface for AsyncInterface<I2C> { ... }
```

Each interface gets its own `Inner` typed wrapper. The `Inner` type
is the `device-driver`-generated `Inner<I>` struct.

### D5. Doctests

Each method on `Tmp108` has its own doctest with the blocking shape:

```rust
/// ```
/// use tmp108::Tmp108;
/// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
/// let i2c = Mock::new(&[
///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
/// ]);
/// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
/// let temp = tmp.temperature().unwrap();
/// assert!((temp - 50.0).abs() < 0.01);
/// # let mut i2c = tmp.destroy();
/// # i2c.done();
/// ```
```

No more `# #[cfg(feature = "async")] fn main() {}` shim.

Each method on `AsyncTmp108` has its own doctest with the async
shape, gated by `#[cfg(feature = "async")]` on the *method's doc
block* (rustdoc treats this automatically since the method is gated):

```rust
/// ```
/// # tokio::runtime::Runtime::new().unwrap().block_on(async {
/// use tmp108::AsyncTmp108;
/// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
/// let i2c = Mock::new(&[
///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
/// ]);
/// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
/// let temp = tmp.temperature().await.unwrap();
/// assert!((temp - 50.0).abs() < 0.01);
/// # let mut i2c = tmp.destroy();
/// # i2c.done();
/// # });
/// ```
```

Async doctests run only under `cargo test --doc -F async`. The
existing `tokio` dev-dependency supports this; `continuous()` already
uses this exact pattern today.

### D6. `AGENTS.md` updates

- **Gotcha 3** (`maybe-async-cfg` doctest shim): removed entirely.
- **Gotcha 4** (`AsyncFnOnce` for `continuous`): kept (intrinsic to
  the API).
- **Feature-flag matrix**: update to show the two struct names and
  the new `embedded-sensors-hal-async` → `async` implicit dep.
- **"What this crate is"**: rewrite to describe two driver types
  instead of one.

### D7. `Error` and `Config` derive cleanups

- `Config`: derive `Eq` (all fields are C-style enums; free).
- `Error<E, P>`: keep `Debug`. Do **not** add `Clone`/`Copy`/`Eq`
  unconditionally — `E` and `P` may not implement them. A conditional
  derive (`#[derive(Clone)]` with manual `impl<E: Clone, P: Clone>
  Clone for Error<E, P>`) is straightforward and useful. Add:
  - `Clone`: when `E: Clone, P: Clone`
  - `Copy`: when `E: Copy, P: Copy`
  - `PartialEq`: when `E: PartialEq, P: PartialEq`
  - `Eq`: when `E: Eq, P: Eq`

  All four can be manual impls; no derive trickery needed.

### D8. `AlertTmp108` cleanup

- Make `tmp108` field private.
- Add `pub fn into_inner(self) -> (AsyncTmp108<I2C>, ALERT)` (mirrors
  the existing `into_alert` on the bare driver).
- Add `pub fn sensor(&self) -> &AsyncTmp108<I2C>` and
  `pub fn sensor_mut(&mut self) -> &mut AsyncTmp108<I2C>` for read
  and write access (replacement for the public field). These names
  follow the embedded-driver convention.
- `destroy()` stays unchanged: `(I2C, ALERT)`.

### D9. Tests

- The two existing `tests::blocking` and `tests::asynchronous`
  submodules stay, but each calls its own concrete type
  (`Tmp108::new_*` vs `AsyncTmp108::new_*`). Wire-level mock
  expectations stay byte-for-byte identical.
- A new `tests` submodule for `mod ops` tests covers the pure
  functions directly (no mock needed): `to_celsius`, `to_raw` range
  edge cases, `snap_hysteresis` boundary, etc.
- `tests/reexports.rs` updated to also pin `AsyncTmp108` and the
  new `AlertTmp108::sensor` / `into_inner` surface.

### D10. Examples

- `examples/oneshot.rs`: uses blocking `Tmp108` (or async
  `AsyncTmp108` under `-F async`). Existing file already cfg-gates;
  rewrite to import the right name per cfg.
- `examples/continuous.rs`: uses `AsyncTmp108`. Rename in body.
- `examples/sensor_trait.rs`: uses `Tmp108` (blocking trait) or
  `AsyncTmp108` (async trait) depending on feature. Existing file
  already cfg-gates.
- `examples/alert_interrupt.rs`, `examples/alert_comparator.rs`: use
  `AlertTmp108` (unchanged) but the inner `tmp108` field access
  becomes `sensor_mut()`.
- `README.md` snippets follow the example sources verbatim per
  `scripts/check-readme-snippets.sh`. The snippet marker regions are
  rewritten in the examples; the README is regenerated from the
  examples.

---

## Public-API impact summary

### Breaking

| Symbol | Before (0.6.0) | After (0.7.0) |
|---|---|---|
| Async driver type | `Tmp108<I2C>` under `feature="async"` | `AsyncTmp108<I2C>` under `feature="async"` |
| `AlertTmp108::tmp108` field | `pub` | private; use `sensor()` / `sensor_mut()` |
| `embedded-sensors-hal-async` feature | did not imply `async` | now implies `async` |

### Non-breaking additions

| Symbol | Purpose |
|---|---|
| `AlertTmp108::sensor` / `sensor_mut` | replace `pub tmp108` field |
| `AlertTmp108::into_inner` | round-trip from `AlertTmp108` to `(AsyncTmp108, ALERT)` |
| `Eq` on `Config` | free derive |
| `Clone`/`Copy`/`PartialEq`/`Eq` on `Error<E, P>` (conditional) | useful for testing |

### Internal (non-API)

- `maybe-async-cfg` dependency removed.
- New private `mod ops`.
- New private `struct AsyncInterface<I2C>`.

---

## Migration notes (for CHANGELOG.md)

```
## 0.7.0

### Breaking

- The async driver type has been renamed: `Tmp108<I2C>` under
  `feature = "async"` is now `AsyncTmp108<I2C>`. The blocking
  `Tmp108<I2C>` is unchanged. Both types are now available
  simultaneously when both relevant features are enabled.
- `AlertTmp108::tmp108` is no longer a public field. Use the new
  `sensor()` / `sensor_mut()` accessors, or `into_inner()` to
  destructure into `(AsyncTmp108<I2C>, ALERT)`.
- The `embedded-sensors-hal-async` feature now implicitly enables
  `async`. Callers that listed both explicitly are unaffected; those
  who relied on enabling `embedded-sensors-hal-async` without `async`
  were silently building dead code and should remove the now-redundant
  feature gates around their alert handling.

### Added

- `AlertTmp108::sensor()` / `sensor_mut()` accessors.
- `AlertTmp108::into_inner(self) -> (AsyncTmp108<I2C>, ALERT)`.
- `Eq` on `Config`.
- Conditional `Clone`/`Copy`/`PartialEq`/`Eq` on `Error<E, P>`.

### Internal

- The driver no longer depends on `maybe-async-cfg`. The two driver
  types are written out directly; shared register encoding/decoding
  lives in a private `mod ops`.
- Async paths now have first-class doctest coverage.
```

---

## Commit plan (bisectable, one concern per commit)

Each commit lands a self-contained step. Each commit must:
- Build clean under all 8 feature combinations (`cargo hack`).
- Pass `cargo clippy --all-features --all-targets` pedantic.
- Pass `cargo +nightly fmt --check`.
- Pass `cargo test` at applicable feature levels.
- Have the `Assisted-by:` trailer per AGENTS.md.

The order is carefully chosen so each commit is self-contained:

1. **`docs: add design spec for type-split refactor (0.7.0)`**
   Just this spec file. No code touched.

2. **`refactor: extract pure register codec to private mod ops`**
   Move `to_celsius`, `to_raw`, the hysteresis snapping, and the
   `Config <-> register-bytes` conversions into a new `mod ops`.
   Both the blocking and async sides of `maybe-async-cfg` are
   updated to call into `ops::*`. No new feature, no new type, no
   API change. The driver is mechanically simpler but functionally
   identical. Unit tests for the pure functions are added in the same
   commit.

3. **`refactor: introduce AsyncInterface, the async-side wire layer`**
   Split the internal `Interface` into two parallel structs:
   `Interface` (blocking, always available) and `AsyncInterface`
   (`#[cfg(feature = "async")]`). Remove the `maybe-async-cfg`
   wrapper around the interface and its `RegisterInterface` /
   `AsyncRegisterInterface` impls. Drop the `maybe-async-cfg`
   wrapper from the `struct Tmp108` declaration too (but it still
   wraps the methods at this point). Update the `Tmp108<I2C: I2c>`
   bound to allow both shapes — using a conditional bound via
   `cfg(feature = "async")` switching between the two trait names is
   awkward, so this step actually requires step 4 to land at the
   same time. **Merge with step 4 below; this is not landable on
   its own.**

   **Revised:** combine steps 3 and 4. See step 3' below.

3'. **`refactor: introduce AsyncTmp108 alongside Tmp108, drop maybe-async-cfg`**
    The big mechanical commit. After this lands:
    - `pub struct Tmp108<I2C: embedded_hal::i2c::I2c>` is the blocking
      driver, always available.
    - `pub struct AsyncTmp108<I2C: embedded_hal_async::i2c::I2c>` is
      the async driver, `#[cfg(feature = "async")]`.
    - Internal `Interface` (blocking) and `AsyncInterface` (async)
      replace the `maybe-async-cfg`'d Interface.
    - All public methods exist on both types with parallel bodies
      that delegate to `ops::*` for shared logic.
    - All trait impls (`embedded-sensors-hal*`) are updated to target
      the right concrete type:
      - `embedded_sensors_hal::TemperatureSensor for Tmp108<I2C>` (blocking).
      - `embedded_sensors_hal_async::TemperatureSensor for AsyncTmp108<I2C>` (async).
      - All `TemperatureThresholdSet` / `TemperatureHysteresis` impls
        on the bare driver target `AsyncTmp108`.
    - `AlertTmp108<I2C, ALERT>` wraps `AsyncTmp108<I2C>` (its field
      type changes; visibility is still `pub` at this step — that
      change comes in step 5).
    - `into_alert` on the blocking `Tmp108` is removed (it never made
      sense; it required async features); only `AsyncTmp108` has
      `into_alert`.
    - `Cargo.toml` removes the `maybe-async-cfg` dependency.
    - `mod tests::blocking` calls `Tmp108`; `mod tests::asynchronous`
      calls `AsyncTmp108`. Wire expectations are byte-identical.
    - Doctests on `Tmp108` lose the `# #[cfg(feature = "async")] fn main()`
      shim. Doctests on `AsyncTmp108` use the `tokio::block_on` wrapper.
    - `tests/reexports.rs` adds `AsyncTmp108` to the pinned list.
    - The `embedded-sensors-hal-async` feature now lists `async` as a
      dep in `Cargo.toml`.

    This is the heart of the refactor. It is large but mechanical and
    each constituent change is local to a single declaration.

4. **`feat!: make AlertTmp108::tmp108 field private; add sensor() / into_inner()`**
    - `tmp108: AsyncTmp108<I2C>` becomes private.
    - Add `pub fn sensor(&self) -> &AsyncTmp108<I2C>`.
    - Add `pub fn sensor_mut(&mut self) -> &mut AsyncTmp108<I2C>`.
    - Add `pub fn into_inner(self) -> (AsyncTmp108<I2C>, ALERT)`.
    - Update `examples/alert_interrupt.rs` and
      `examples/alert_comparator.rs` to use `sensor_mut()`.
    - Update the existing `handle_threshold_alerts_properly` and
      `alert_pin_error_is_propagated_as_error_pin` tests to use
      `sensor_mut()`.
    - `tests/reexports.rs` pins `sensor`, `sensor_mut`, `into_inner`.
    - **BREAKING**.

5. **`feat: add Eq to Config; add conditional trait derives to Error<E, P>`**
    - `#[derive(... Eq)]` on `Config`.
    - Hand-written `impl<E, P> Clone/Copy/PartialEq/Eq for Error<E, P>`
      with the appropriate bounds.
    - Tests assert that `assert_eq!(default_cfg, default_cfg)`
      compiles and `Error::InvalidInput == Error::InvalidInput` works.

6. **`docs: update AGENTS.md for the type-split architecture`**
    - Remove Gotcha 3 (`maybe-async-cfg` doctest shim).
    - Update "What this crate is" to describe two driver types.
    - Update "Feature-flag matrix" to show the new explicit `async`
      dep on `embedded-sensors-hal-async` and the two struct names.
    - Update "Where to put new things" — adding a new public method
      now means **two parallel additions**, one on each type, with
      shared logic in `mod ops`.
    - The "Required reading" pointer to the old design spec is
      preserved; add a pointer to this one.

7. **`docs: rewrite README snippets to use AsyncTmp108`**
    Update the snippet markers in `examples/continuous.rs` and
    `examples/alert_interrupt.rs`, then sync the README's snippet
    blocks. `scripts/check-readme-snippets.sh` must pass.

8. **`chore: bump version to 0.7.0 and update CHANGELOG`**

---

## Risks

1. **Combined-commit step 3' is large.** ~1700 lines of `src/lib.rs`
   are rewritten. Mitigations:
   - `mod ops` is already extracted in step 2, shrinking the diff.
   - The wire-level mock expectations don't change, providing a
     strong invariant: any test failure after step 3' is a real bug
     in the rewrite, not a behavior change.
   - Each "shell" body is small (~5 lines) and parallel to its
     counterpart — review can scan them in pairs.

2. **`AsyncInterface` symbol naming.** `device-driver` generates an
   `Inner<I>` wrapper that takes the interface as its type parameter.
   The two driver types each hold their own `Inner<Interface<I2C>>`
   or `Inner<AsyncInterface<I2C>>`. The generated `Inner` is the same
   type for both, parameterized differently. No conflict; the
   distinction is by the interface type.

3. **Doctest cost.** Adding doctests on every `AsyncTmp108` method
   doubles the doctest count. Each doctest spins up a `tokio` runtime
   — they run fast individually but the total adds up. Acceptable;
   `cargo test --doc -F async` is on the canonical CI matrix.

4. **`maybe-async-cfg` removal.** The dependency is currently a
   regular `[dependencies]` entry (per AGENTS.md Gotcha 10 — it's
   needed at expansion time, which technically makes it a
   build-dep-flavored runtime-dep). Removing it deletes the entry
   and dozens of `#[maybe_async_cfg::maybe(...)]` attribute lines.
   `cargo-vet` will silently approve the removal (fewer deps = fewer
   audits needed).

5. **`embedded-sensors-hal-async` feature now requires `async`.** This
   is a *strengthening* of the requirement — the old combo "async-off,
   sensors-hal-async-on" was meaningless dead code. The CHANGELOG
   calls this out. CI's `cargo hack --feature-powerset` will exercise
   the new combos automatically.

6. **`destroy()` on `AlertTmp108` returns `(I2C, ALERT)` not
   `(AsyncTmp108<I2C>, ALERT)`.** Already the case in 0.6.0; not
   changed here. The new `into_inner()` returns the typed wrapper if
   the caller wants it.

---

## Out of scope for this PR (tracked for follow-up)

- Adding new methods to either type beyond `AlertTmp108::sensor*` /
  `into_inner` (existing surface preserved).
- Probe-on-construct (`new` becoming fallible) — already rejected in
  PR1.
- Hardware verification (H3 from PR1's review).
- Diagnostic counters (L5 from PR1's review).
- Reworking the `Error` enum further (e.g., adding `Error::Compound`).
- Adding `embedded-storage-async`-style "Bus" abstraction layer — the
  shared `mod ops` already captures the right amount of sharing for
  this 4-register chip; a `Bus` trait would add layers without
  payoff.
