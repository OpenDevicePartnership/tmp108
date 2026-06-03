# tmp108 — Compile-verified usage docs for AI agents

**Date:** 2026-06-03
**Status:** Approved (design)
**Audience:** Maintainers, and the implementer (human or agent) who will execute
the plan that comes from this spec.

---

## 1. Goal

An AI agent reading docs.rs or `examples/` for `tmp108` gets snippets that:

1. **Compile against the real API.** Today's README does not.
2. **Cover every workflow the chip supports.** Today, only one is shown.
3. **Cannot drift again** — CI fails when documentation diverges from the API.

Secondary goal: a human reading the same material gets a usable cookbook for the
four operating modes of the part, including the alert state machine, without
owning an Embassy/RTIC project.

## 2. Non-goals

- No Embassy, RTIC, or MCU-specific HAL integration. The abstraction is
  `embedded-hal` (blocking) and `embedded-hal-async`, with `pico-de-gallo-hal`
  as the concrete implementation used to run examples.
- No new public methods on `Tmp108` or `AlertTmp108`. Pure docs + examples + CI
  + one minimal re-export change (see §3.0).
- No `AGENTS.md` in this round. Worth a follow-on; out of scope here.
- No reorganization of the existing test module.

## 3. Deliverables

### 3.0 Re-export `Config` field types (prerequisite)

`Config`'s fields are public, but the types they hold (`Mode`, `Thermostat`,
`ConversionRate`, `Hysteresis`, `Polarity`) are imported into `src/lib.rs` with
`use` rather than `pub use` (`src/lib.rs:35`). Result: downstream consumers can
read `Config` but cannot construct one with non-default values. Verified by
attempting to build `Config { thermostat_mode: tmp108::Thermostat::Interrupt,
.. }` from an out-of-crate test, which fails with `E0603: enum 'Thermostat' is
private`.

**Change:** add `pub use crate::inner::{ConversionRate, Hysteresis, Mode,
Polarity, Thermostat};` to the crate root. One line. Semver-additive (no
existing item is removed or renamed). Unblocks every Config-using example and
doctest in §3.2 and §3.3.

### 3.1 README rewrite

Replace the current `## Usage` section in `README.md`. Because `src/lib.rs:12`
does `#![doc = include_str!("../README.md")]`, the same content becomes the
crate-level rustdoc on docs.rs.

The rewrite contains:

1. **Three real snippets, lifted verbatim from CI-verified sources.** Each
   snippet is short (≤25 lines) and shows one workflow:

   - blocking one-shot read,
   - async continuous loop with `wait_for_temperature`,
   - `AlertTmp108` interrupt-mode threshold wait.

   Snippets are extracted by tooling (see §3.4) so they cannot drift.

2. **Feature-flag matrix** (table): `async`, `embedded-sensors-hal`,
   `embedded-sensors-hal-async`, the legal combinations, and what each
   unlocks. Specifically note that `embedded-sensors-hal-async` requires
   `async`, and `AlertTmp108` requires both.

3. **Gotchas list** (bulleted, concise):

   - Comparator mode latches the ALERT pin until temperature is back inside
     `(T_low + HYS, T_high − HYS)`; interrupt mode clears on config-register
     read.
   - ALERT polarity is configured on-chip; wire your pull resistor
     accordingly. The Pico-de-Gallo examples assume active-low + external
     pull-up.
   - `Tmp108::new` does **not** take a `DelayNs`. The delay only appears where
     it is genuinely needed (`wait_for_temperature`).
   - Raw temperature is a 12-bit signed value in the upper bits of a 16-bit
     register; the driver converts to `f32` Celsius at 0.0625 °C/LSB.
   - `continuous` puts the chip back into shutdown when its closure returns.

### 3.2 Per-method `# Examples` doctests in `src/lib.rs`

Every public method on `Tmp108` and `AlertTmp108` gets a `# Examples` block. All
doctests use `embedded-hal-mock` (already a dev-dep) so `cargo test --doc`
actually executes them.

In scope (concrete list):

- `Tmp108::new`, `new_with_a0_gnd`, `new_with_a0_vplus`, `new_with_a0_sda`,
  `new_with_a0_scl`, `addr`, `destroy`, `into_alert`
- `Tmp108::read_configuration`, `configure`
- `Tmp108::temperature`
- `Tmp108::one_shot`, `shutdown`, `continuous` (async only),
  `wait_for_temperature`
- `Tmp108::low_limit`, `set_low_limit`, `high_limit`, `set_high_limit`
- `AlertTmp108::new`, `new_with_a0_gnd`, `new_with_a0_vplus`, `new_with_a0_sda`,
  `new_with_a0_scl`, `destroy`

Each doctest is 5–15 lines (excluding mock setup), uses `# fn main()` hiding to
keep the visible snippet tight, and asserts at least one expected effect (e.g.,
final mock transaction count, returned address). Async doctests use the `tokio`
test harness already established in the existing test module.

`new_with_a0_*` doctests may be omitted on the trivial constructors if a single
representative doctest on `new` would clearly suffice — implementer's
judgement. Default: include them, agents grep for the exact constructor name.

### 3.3 Expanded `examples/` directory

All examples run on Pico de Gallo with a TMP108 wired as:

- I²C on the default bus, A0 → GND → address `0x48`
- ALERT → GPIO0 on the Pico (external 2 kΩ pull-up to V+)

Every example file begins with a `//!` block declaring: what the program
demonstrates, the required hardware wiring, the cargo feature flags it expects,
and which TMP108 register interactions occur.

| File | Variants | What it shows |
|------|----------|---------------|
| `oneshot.rs` | blocking + async (exists) | Single conversion (light touch; bring docstring into the new style) |
| `continuous.rs` | **async only** | `Tmp108::continuous` closure with `wait_for_temperature(&mut delay)`, loops N times, prints temperatures. Async-only because `Tmp108::continuous` itself is async-only (`src/lib.rs:317`); the `//!` block documents this and points the reader at a blocking-mode pattern using `configure(...)` + `wait_for_temperature` + `shutdown()`. |
| `alert_interrupt.rs` | **async only** | `AlertTmp108` in interrupt mode: set low/high thresholds, await alert via GPIO0 falling edge, print the temperature returned by `wait_for_temperature_threshold`. |
| `alert_comparator.rs` | **async only** | `AlertTmp108` in comparator mode: same wiring, but show the latched-pin behavior — a second call to `wait_for_temperature_threshold` returns immediately while still over-threshold; loop demonstrates the hysteresis-driven release. |
| `sensor_trait.rs` | blocking + async | A generic function over `embedded_sensors_hal::TemperatureSensor` / `embedded_sensors_hal_async::temperature::TemperatureSensor` is called with a `Tmp108`. Demonstrates the integration pattern for larger firmware that wants to abstract over multiple sensors. |

**Async-only justification:** `AlertTmp108` is gated on `feature =
"embedded-sensors-hal-async"` + `feature = "async"`
(`src/lib.rs:143-201`). There is no blocking equivalent in the driver.
`continuous.rs` is async-only because `Tmp108::continuous` itself is gated on
`feature = "async"` (`src/lib.rs:317`).

The blocking/async variants are produced via the `#[cfg(not(feature =
"async"))]` / `#[cfg(feature = "async")]` split already used in
`examples/oneshot.rs`. One file per example, two `main` functions.

**Inert-when-features-off invariant:** `cargo build --examples` always builds
every file in `examples/`, regardless of features. To prevent compile errors in
unsupported feature combinations, every example file uses a top-level cfg guard:

```rust
#![cfg_attr(not(<required features>), allow(unused_imports))]
// then both variants use #[cfg(...)] on their main() so exactly one
// (or neither) is compiled in any given feature combination.
```

For files that have no valid main at all in some combination (e.g.,
`alert_interrupt.rs` with `--no-default-features`), the file uses a single
`#[cfg_attr(not(all(feature = "async", feature = "embedded-sensors-hal-async")),
allow(dead_code))] fn main() {}` stub so `cargo build --examples` always
succeeds.

### 3.4 README snippet extraction

To enforce "snippets are lifted verbatim from CI-verified sources," each README
snippet is bracketed in its source example file with marker comments:

```rust
// README-SNIPPET-START: oneshot
… real code …
// README-SNIPPET-END: oneshot
```

A short build script (`xtask` binary or a plain
`scripts/check-readme-snippets.sh`) extracts each region and compares it to the
matching fenced block in `README.md`. CI runs this check. **Implementer
chooses** the simplest mechanism that works; a 30-line shell/`awk` script is
acceptable.

### 3.5 CI changes (`.github/workflows/check.yml`)

Add to the existing `test` job (or a new `examples` job — implementer's choice):

```yaml
- name: cargo test --doc (blocking)
  run: cargo test --doc --locked

- name: cargo test --doc (async)
  run: cargo test --doc --locked -F async

- name: cargo test --doc (embedded-sensors-hal-async)
  run: cargo test --doc --locked -F async,embedded-sensors-hal-async

- name: cargo build --examples (blocking)
  run: cargo build --examples --locked

- name: cargo build --examples (async)
  run: cargo build --examples --locked -F async

- name: cargo build --examples (embedded-sensors-hal-async)
  run: cargo build --examples --locked -F async,embedded-sensors-hal-async

- name: README snippets in sync
  run: ./scripts/check-readme-snippets.sh
```

The `cargo hack --feature-powerset` job already covers feature unification; we are not duplicating that.

## 4. Constraints

- **MSRV 1.90 and `edition = "2024"`** must continue to pass. Examples and
  doctests must not use newer features.
- **`unsafe_code = "deny"`, `missing_docs = "deny"`, `clippy::pedantic =
  "deny"`** apply across the crate. Examples are part of the crate's target set;
  pedantic clippy applies.
- **Pico-de-Gallo hardware required** for `cargo run --example …` to
  succeed. Confirmed by the user: the board is attached, TMP108 at `0x48`, ALERT
  on GPIO0 with a 2 kΩ pull-up.
- **No new runtime dependencies on the library crate.** `pico-de-gallo-hal` and
  `tokio` are already dev-dependencies.

## 5. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Doctests inflate `cargo test` time. | Keep each ≤15 lines and reuse a single mock-builder pattern. Doctests still run faster than the existing unit tests. |
| README snippet extractor adds maintenance burden. | Use a ≤50-line shell script; document the marker convention in `CONTRIBUTING.md`. |
| Alert examples depend on GPIO0 wiring — agents on other boards may copy and be confused. | Every example's `//!` block states the exact wiring assumed, in the first three lines. |
| Comparator-mode example may be flaky on real hardware (latched pin + ambient temperature). | The example explicitly waits for a user-applied warm/cold stimulus and prints what it observes; it does not assert specific temperature values. |
| `cargo hack --feature-powerset` may try to build `--features embedded-sensors-hal-async` without `async`. | The current `Cargo.toml` already allows this; the new examples are feature-gated to require both, so they won't be built in the impossible combination. |

## 6. Acceptance criteria

A reviewer can verify each of these mechanically.

1. The crate re-exports `ConversionRate`, `Hysteresis`, `Mode`, `Polarity`, and
   `Thermostat` at the crate root; an out-of-crate consumer can construct
   `Config { thermostat_mode: tmp108::Thermostat::Interrupt, ..Default::default()
   }` without errors.
2. `cargo test --doc` passes with no features, with `--features async`, and with
   `--features "async embedded-sensors-hal-async"`.
3. `cargo build --examples` passes for each feature combination listed in §3.5.
4. Every public method on `Tmp108` and `AlertTmp108` (per the list in §3.2) has
   a `# Examples` block visible in `cargo doc --no-deps --all-features --open`.
5. Running `cargo run --example oneshot`, `cargo run --example continuous
   --features async`, `cargo run --example alert_interrupt --features "async
   embedded-sensors-hal-async"`, `cargo run --example alert_comparator
   --features "async embedded-sensors-hal-async"`, and `cargo run --example
   sensor_trait --features embedded-sensors-hal` (and the async variant) all
   execute end-to-end against the attached Pico de Gallo and print sensible
   output.
6. The README's three usage snippets match the marker regions in their source
   example files byte-for-byte; the snippet-check script returns 0.
7. **Hardware verification of the alert state machine:** `alert_interrupt`
   triggers on a finger-warming stimulus and reports a temperature above its
   configured high limit; `alert_comparator` demonstrates the latched-pin
   behavior (a second `wait_for_temperature_threshold` call returns immediately
   while still over-threshold) and the release after the temperature drops below
   `T_high − HYS`. Observed behavior matches the comments at
   `src/lib.rs:602-640`.
8. `cargo fmt --check`, `cargo clippy --all-features --all-targets`, and `cargo
   doc --no-deps --all-features` all pass.

## 7. Out of scope (future work)

- `AGENTS.md` with a register-level state machine table and a "common mistakes"
  knowledge base.
- A reference firmware example crate (Embassy/RTIC on a specific MCU).
- A `defmt`-based logging variant of the continuous example for `no_std` users.
- Property-based tests over the `to_raw` / `to_celsius` round-trip.
