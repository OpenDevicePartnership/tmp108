# tmp108 — Compile-verified usage docs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make tmp108 trivially correct to use from an AI agent's perspective:
fix the misleading README, add CI-verified doctests on every public method, and
add Pico-de-Gallo-based examples covering all four chip workflows.

**Architecture:** Tiny library + dev-only changes. One source change to the
library (a `pub use` line). Five examples, each runnable on a real Pico de
Gallo with TMP108 at `0x48` and ALERT on GPIO0. Every snippet that appears in
the README is bracketed in its source example with marker comments and a CI
script enforces byte-equality. Doctests use `embedded-hal-mock` (existing
dev-dep), so they execute under `cargo test --doc` with no hardware required.

**Tech Stack:** Rust 2024, MSRV 1.90; `embedded-hal` 1.0 / `embedded-hal-async`
1.0; `device-driver` 1.0.9; `pico-de-gallo-hal` 0.5 / `pico-de-gallo-lib` 0.5;
`embedded-hal-mock` 0.11; `tokio` (dev-only, multi-thread runtime for async
examples).

**Spec:** `docs/superpowers/specs/2026-06-03-tmp108-agent-usage-docs-design.md`

**Conventions used throughout this plan:**
- All paths are repo-relative (root: `/home/balbi/workspace/odp/tmp108`).
- `git commit -m "..."` is shown literally — copy as-is.
- Test commands assume default features unless `-F` is shown.
- `cargo run --example NAME` examples require the Pico de Gallo attached at
  USB id `045e:067d` with the TMP108 reachable at `0x48` and ALERT on GPIO0
  with an external 2 kΩ pull-up.

---

## File Map

| File | Purpose | Created / Modified |
|------|---------|--------------------|
| `Cargo.toml` | Bump `pico-de-gallo-hal` dev-dep to 0.5, add `pico-de-gallo-lib` dev-dep. | Modified |
| `src/lib.rs` | Add `pub use` re-exports for `ConversionRate`, `Hysteresis`, `Mode`, `Polarity`, `Thermostat`. Add `# Examples` doctests on every public `Tmp108`/`AlertTmp108` method. Replace the broken README. | Modified |
| `README.md` | Rewrite with three CI-verified snippets (lifted via marker regions), feature-flag matrix, gotchas list. | Modified |
| `examples/oneshot.rs` | Rework `//!` header to the new style; bracket README region. | Modified |
| `examples/continuous.rs` | New, async-only. `Tmp108::continuous` with `wait_for_temperature`. Brackets the README "continuous" region. | Created |
| `examples/alert_interrupt.rs` | New, async-only. `AlertTmp108` interrupt mode on GPIO0. Brackets the README "alert" region. | Created |
| `examples/alert_comparator.rs` | New, async-only. `AlertTmp108` comparator mode, demonstrates latched-pin + hysteresis release. | Created |
| `examples/sensor_trait.rs` | New, blocking + async. Generic function over the `embedded-sensors-hal` traits. | Created |
| `scripts/check-readme-snippets.sh` | New. Extracts README-SNIPPET regions and diffs them against the matching fenced code blocks in `README.md`. | Created |
| `.github/workflows/check.yml` | Extend the `test` job to run `cargo test --doc` for three feature sets, `cargo build --examples` for three feature sets, and the snippet check. | Modified |
| `CONTRIBUTING.md` | Add a short "README snippet markers" section pointing at `scripts/check-readme-snippets.sh`. | Modified |

---

## Task 0: Bump pico-de-gallo-hal dev-dep and add pico-de-gallo-lib

`pico-de-gallo-hal` 0.1 hangs on USB I/O against the firmware currently on the
attached board; 0.5 works (verified live). The alert examples need
`GpioDirection`/`GpioPull` from `pico-de-gallo-lib` (0.5) to configure GPIO0
as an input. Cargo.toml is currently in the working tree showing the bump but
not committed.

**Files:**
- Modify: `Cargo.toml`
- Modify: `Cargo.lock` (auto-regenerated)

- [ ] **Step 1: Confirm working tree already has the bump from prior session**

Run: `git diff Cargo.toml`
Expected:
```diff
-pico-de-gallo-hal = "0.1.0"
+pico-de-gallo-hal = "0.5.0"
```
If diff is empty, manually edit `Cargo.toml` to set `pico-de-gallo-hal = "0.5.0"`.

- [ ] **Step 2: Add `pico-de-gallo-lib` as a dev-dependency**

Edit `Cargo.toml`, in the `[dev-dependencies]` block, add directly after the
`pico-de-gallo-hal` line:

```toml
pico-de-gallo-lib = "0.5.0"
```

- [ ] **Step 3: Refresh the lock file**

Run: `cargo update -p pico-de-gallo-hal -p pico-de-gallo-lib --locked` (allowed
to fail; falls back to plain update)
Then: `cargo build --tests`
Expected: builds cleanly.

- [ ] **Step 4: Confirm the existing example still runs against the board**

Run: `cargo run --example oneshot`
Expected: prints `Temperature: <number> C` and exits 0.

- [ ] **Step 5: Commit**

```bash
git add Cargo.toml Cargo.lock
git commit -m "build: bump pico-de-gallo-hal dev-dep to 0.5 + add pico-de-gallo-lib

The 0.1 release hangs on USB I/O against the firmware now shipped on
Pico de Gallo. 0.5 works and is the version examples target. Adding
pico-de-gallo-lib gives examples access to GpioDirection / GpioPull
for configuring GPIO0 as the ALERT input pin."
```

---

## Task 1: Re-export Config field types from the crate root

Without this, examples and out-of-crate doctests cannot construct a non-default
`Config`. Verified by an out-of-crate test crate that hits `E0603` when writing
`Config { thermostat_mode: tmp108::Thermostat::Interrupt, .. }`. This is the
single library change in this plan.

**Files:**
- Modify: `src/lib.rs:35`

- [ ] **Step 1: Write the failing out-of-crate test**

Create `tests/reexports.rs`:

```rust
//! Compile-only test: Config field types must be reachable from outside the crate.
use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};

#[test]
fn config_with_explicit_fields_compiles() {
    let cfg = Config {
        thermostat_mode: Thermostat::Interrupt,
        alert_polarity: Polarity::ActiveHigh,
        conversion_rate: ConversionRate::_16Hz,
        hysteresis: Hysteresis::_4C,
    };
    // Avoid unused-binding warning; the assertion is on compilation.
    let _ = cfg;
}
```

- [ ] **Step 2: Run the test, confirm it fails**

Run: `cargo test --test reexports`
Expected: `error[E0432]: unresolved imports tmp108::ConversionRate, tmp108::Hysteresis, tmp108::Polarity, tmp108::Thermostat`

- [ ] **Step 3: Add the re-exports**

Edit `src/lib.rs` line 35, replacing:

```rust
use crate::inner::{ConversionRate, Hysteresis, Inner, Mode, Polarity, Thermostat};
```

with:

```rust
use crate::inner::Inner;

pub use crate::inner::{ConversionRate, Hysteresis, Mode, Polarity, Thermostat};
```

The split keeps `Inner` private (it's an implementation detail) while making
the user-facing enums public. `Mode` is also re-exported even though it isn't
currently a field on `Config`; it is the type returned by/passed to
`one_shot`/`shutdown`/`continuous` indirectly and downstream users may
reasonably want to name it.

- [ ] **Step 4: Run the test again, confirm it passes**

Run: `cargo test --test reexports`
Expected: `test config_with_explicit_fields_compiles ... ok`

- [ ] **Step 5: Confirm existing tests still pass on all relevant feature sets**

Run, sequentially:
```
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```
Expected: all pass.

- [ ] **Step 6: Confirm semver-checks are happy (additive change)**

Run: `cargo install --locked cargo-semver-checks --quiet 2>&1 | tail -1 || true; cargo semver-checks check-release --baseline-rev HEAD~0 2>&1 | tail -10`
Expected: `Summary: no errors, no warnings` (or equivalent "no breaking changes"
output). If `cargo-semver-checks` isn't installable, skip this step — CI runs
it.

- [ ] **Step 7: Commit**

```bash
git add src/lib.rs tests/reexports.rs
git commit -m "lib: re-export Config field types from crate root

Mode, Thermostat, ConversionRate, Hysteresis, and Polarity are the
types of Config's public fields. Without 'pub use', downstream
consumers can read a Config but cannot construct one with non-default
values - 'Config { thermostat_mode: tmp108::Thermostat::Interrupt, .. }'
fails with E0603. Added a compile-only test in tests/reexports.rs to
pin this behavior."
```

---

## Task 2: Add `# Examples` doctests on `Tmp108` constructors and accessors

This and Tasks 3-5 add doctests. Pattern: each doctest uses
`embedded-hal-mock`'s I2C `Mock` with explicit `Transaction` expectations so
the doctest both compiles *and* demonstrates the chip's wire-level interaction.
Hidden lines (`# `) keep the rendered snippet tight.

**Files:**
- Modify: `src/lib.rs:99-150` (the impl block with `new`, `new_with_a0_*`, `addr`, `destroy`, `into_alert`)

- [ ] **Step 1: Add `# Examples` to `Tmp108::new`**

Replace `src/lib.rs:100-106`:

```rust
    /// Create a new TMP108 instance.
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = Interface::new(i2c, a0);
        let inner = Inner::new(interface);

        Self { inner }
    }
```

with:

```rust
    /// Create a new TMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::{A0, Tmp108};
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new(i2c, A0::Sda);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = Interface::new(i2c, a0);
        let inner = Inner::new(interface);

        Self { inner }
    }
```

- [ ] **Step 2: Add `# Examples` to `new_with_a0_gnd`**

Replace `src/lib.rs:108-112`:

```rust
    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }
```

with:

```rust
    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }
```

- [ ] **Step 3: Add `# Examples` to `new_with_a0_vplus`, `new_with_a0_sda`, `new_with_a0_scl`**

Same pattern as Step 2 but with `Vplus`/`Sda`/`Scl` and expected addresses
`0x49`/`0x4a`/`0x4b` respectively. The full code (don't read this as a
placeholder — write each one):

```rust
    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_vplus(i2c);
    /// assert_eq!(tmp.addr(), 0x49);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_sda(i2c);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_scl(i2c);
    /// assert_eq!(tmp.addr(), 0x4b);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }
```

- [ ] **Step 4: Add `# Examples` to `addr` and `destroy`**

`addr` already has the assertion shown in Steps 1-3 demonstrating it implicitly,
but agents grep for the exact method name. Add explicit doctests:

```rust
    /// Get the current I2C address
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub fn addr(&self) -> u8 {
        self.inner.interface.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut i2c = tmp.destroy();
    /// i2c.done();
    /// # }
    /// ```
    pub fn destroy(self) -> I2C {
        self.inner.interface.i2c
    }
```

- [ ] **Step 5: Add `# Examples` to `into_alert`**

`into_alert` is gated on `cfg(all(feature = "embedded-sensors-hal-async",
feature = "async"))`. Its doctest must be similarly gated:

```rust
    /// Create a new `AlertTmp108` instance by consuming the original Tmp108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let alert_tmp = tmp.into_alert(alert);
    /// let (mut i2c, mut alert) = alert_tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # });
    /// ```
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, ALERT> {
        AlertTmp108 { tmp108: self, alert }
    }
```

- [ ] **Step 6: Run doctests**

Run sequentially:
```
cargo test --doc --locked
cargo test --doc --locked -F async
cargo test --doc --locked -F async,embedded-sensors-hal-async
```
Expected: every doctest passes. Pay attention to the doctest count
(`X passed`); it should grow by 8 doctests (`new`, four `new_with_a0_*`,
`addr`, `destroy`, `into_alert`).

- [ ] **Step 7: Commit**

```bash
git add src/lib.rs
git commit -m "docs: add # Examples doctests to Tmp108 constructors and accessors

new, new_with_a0_gnd/_vplus/_sda/_scl, addr, destroy, into_alert.
Each example uses embedded-hal-mock so 'cargo test --doc' actually
runs them in CI."
```

---

## Task 3: Add `# Examples` doctests to `Tmp108` configuration methods

Methods covered here: `read_configuration`, `configure`, `temperature`,
`one_shot`, `shutdown`, `continuous`, `wait_for_temperature`.

**Files:**
- Modify: `src/lib.rs:219-357` (impl block containing these methods)

- [ ] **Step 1: Add doctest to `read_configuration`**

The mock expects a single `write_read(addr=0x48, [0x01], [0x22, 0x10])` —
register address 1 (configuration), default value `0x1022` returned in LE byte
order. Confirmed against `tmp108.toml` (default reset_value `0x1022`,
`default_byte_order = "LE"`).

Replace `src/lib.rs:215-232` (the `read_configuration` definition) with:

```rust
    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::{Config, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let cfg = tmp.read_configuration().unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
```

Leave the rest of the method body unchanged. Because the impl block is
`maybe_async_cfg`'d, the doctest must work in the blocking variant; the
generated blocking `read_configuration` returns `Result<Config, _>` directly
(no `.await`). That's why the doctest uses `.unwrap()` and no `.await`.

**Important:** because of `maybe_async_cfg`, this doctest as written tests the
blocking variant only. We test the async variant by adding `# fn main() {}` and
showing the async signature in a separate `ignore`-style doctest, OR by gating
the entire example block with `#[cfg(not(feature = "async"))]` doc-block markup.
**Use the simpler approach:** the doctest text is correct for blocking; the
async variant is exercised by the example in Task 6 and by the existing async
unit test in `src/lib.rs:996`. State this in a brief docstring sentence:

Add this line above `# Examples`:

```rust
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
```

- [ ] **Step 2: Add doctest to `configure`**

Replace the `configure` definition (`src/lib.rs:234-261`) similarly:

```rust
    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.configure(Config {
    ///     thermostat_mode: Thermostat::Interrupt,
    ///     alert_polarity: Polarity::ActiveHigh,
    ///     conversion_rate: ConversionRate::_16Hz,
    ///     hysteresis: Hysteresis::_4C,
    /// }).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
```

The expected wire bytes `[0x01, 0x66, 0xb0]` were verified by the existing
blocking unit test `change_configuration` (`src/lib.rs:799-834`); they
correspond to writing `Configuration` register with `Thermostat::Interrupt`,
`Polarity::ActiveHigh`, `ConversionRate::_16Hz`, `Hysteresis::_4C`.

- [ ] **Step 3: Add doctest to `temperature`**

```rust
    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let temp = tmp.temperature().unwrap();
    /// assert!((temp - 50.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
```

The byte sequence `[0x32, 0x00]` → 50.0 °C is verified by the existing
`read_temperature_default_address` test (`src/lib.rs:842-851`).

- [ ] **Step 4: Add doctests to `one_shot` and `shutdown`**

Both methods modify only the M bits of the configuration register, so the
modify-cycle is `write_read(0x01, ...) → write(0x01, ...)`. Default config is
`0x1022` (M=Continuous=0b10). Setting OneShot=0b01 yields `0x1021`; setting
Shutdown=0b00 yields `0x1020`. Both encoded little-endian.

```rust
    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x21, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.one_shot().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn one_shot(&mut self) -> Result<(), I2C::Error> {
```

```rust
    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.shutdown().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
```

- [ ] **Step 5: Add doctest to `continuous` (async-only)**

`continuous` is gated `#[cfg(feature = "async")]` and is async-only by API
shape. The doctest must therefore use a tokio runtime.

```rust
    #[cfg(feature = "async")]
    /// Initiate continuous conversions
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     // Enter continuous: read cfg, modify M bits, write back.
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
    ///     // Inside the closure: read temperature once.
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    ///     // continuous() returns the chip to shutdown on exit.
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.continuous(|t| async {
    ///     let _ = t.temperature().await?;
    ///     Ok(())
    /// }).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn continuous<F, Fut>(&mut self, f: F) -> Result<(), I2C::Error>
```

Note: the cfg-gating of doctests on `cfg(feature = "async")` works because
rustdoc honors the same cfg attributes. The doctest only runs when
`cargo test --doc -F async`. **Verify this in Step 7 below.**

- [ ] **Step 6: Add doctest to `wait_for_temperature`**

```rust
    /// Wait for conversion to complete. This method will block for the amount
    /// of time dictated by the CR bits in the `Configuration` register.
    /// Caller is required to call this method from within their continuous
    /// conversion closure.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the calls.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::delay::NoopDelay;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut delay = NoopDelay::new();
    /// let temp = tmp.wait_for_temperature(&mut delay).unwrap();
    /// assert!((temp - 50.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn wait_for_temperature<DELAY: AsyncDelayNs>(&mut self, delay: &mut DELAY) -> Result<f32, I2C::Error> {
```

This also fixes the existing broken intra-doc link warning
(`[`Configuration`]` → `` `Configuration` ``).

- [ ] **Step 7: Run doctests**

```
cargo test --doc --locked
cargo test --doc --locked -F async
cargo test --doc --locked -F async,embedded-sensors-hal-async
```
Expected: all pass. The async-only `continuous` doctest must:
- be **skipped** under plain `cargo test --doc` (no `-F async`),
- **run and pass** under `-F async`.

If the cfg-gating doesn't work as expected, fall back to wrapping the
problematic doctest body in `#[cfg(feature = "async")]` inside a `# `-hidden
line and keep the doctest behind a `compile_fail` or `no_run` annotation. Pick
whichever passes; document the choice in the commit message.

- [ ] **Step 8: Commit**

```bash
git add src/lib.rs
git commit -m "docs: add # Examples doctests to Tmp108 config/IO methods

read_configuration, configure, temperature, one_shot, shutdown,
continuous, wait_for_temperature. Mock expectations were cross-checked
against the existing unit tests' wire-level transactions.

Also fixes the broken intra-doc link to [Configuration] on
wait_for_temperature by demoting it to a plain code span."
```

---

## Task 4: Add `# Examples` doctests to `Tmp108` threshold methods

**Files:**
- Modify: `src/lib.rs:359-419` (`low_limit`, `set_low_limit`, `high_limit`, `set_high_limit`)

- [ ] **Step 1: Add doctest to `set_low_limit` / `low_limit`**

Wire encoding: 25.0 °C → raw `0x1900` (existing tests, `src/lib.rs:868-911`).

Replace the `set_low_limit` def with:

```rust
    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_low_limit(25.0).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
```

Replace the `low_limit` def with:

```rust
    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.low_limit().unwrap();
    /// assert!((limit - 25.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn low_limit(&mut self) -> Result<f32, I2C::Error> {
```

- [ ] **Step 2: Add doctest to `set_high_limit` / `high_limit`**

Same pattern as Step 1 but using register address `0x03` and a different
temperature. Use 80.0 °C → raw `0x5000` (existing tests confirm).

```rust
    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.high_limit().unwrap();
    /// assert!((limit - 80.0).abs() < 0.01);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn high_limit(&mut self) -> Result<f32, I2C::Error> {
```

```rust
    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// (Doctest runs against the blocking API; the async variant has the same
    /// shape with `.await` after the call.)
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// # fn main() {
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_high_limit(80.0).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # }
    /// ```
    pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
```

- [ ] **Step 3: Run doctests**

```
cargo test --doc --locked
cargo test --doc --locked -F async
```
Expected: all pass; doctest count grew by 4.

- [ ] **Step 4: Commit**

```bash
git add src/lib.rs
git commit -m "docs: add # Examples doctests to Tmp108 threshold methods

low_limit, set_low_limit, high_limit, set_high_limit. Wire bytes
verified against the existing set_and_read_*_limit unit tests."
```

---

## Task 5: Add `# Examples` doctests to `AlertTmp108`

**Files:**
- Modify: `src/lib.rs:163-201` (the `AlertTmp108` impl block: `new`, `new_with_a0_*`, `destroy`)

- [ ] **Step 1: Add doctest to `AlertTmp108::new` and constructors**

All of these methods are gated `#[cfg(all(feature = "embedded-sensors-hal-async",
feature = "async"))]`. Doctests must use a tokio runtime.

Add doctests for `new`, `new_with_a0_gnd`, `new_with_a0_vplus`,
`new_with_a0_sda`, `new_with_a0_scl`, `destroy`. Use the same shape as Task 2;
example for `new_with_a0_gnd`:

```rust
    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// assert_eq!(tmp.tmp108.addr(), 0x48);
    /// let (mut i2c, mut alert) = tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Gnd, alert)
    }
```

Repeat for `new`, `new_with_a0_vplus`, `new_with_a0_sda`, `new_with_a0_scl`,
`destroy` with the appropriate address assertions and constructor calls. Each
doctest is 8-10 lines.

- [ ] **Step 2: Run doctests**

```
cargo test --doc --locked -F async,embedded-sensors-hal-async
```
Expected: all pass; doctest count grew by 6.

- [ ] **Step 3: Commit**

```bash
git add src/lib.rs
git commit -m "docs: add # Examples doctests to AlertTmp108

new, new_with_a0_gnd/_vplus/_sda/_scl, destroy. Each test sets up an
I2C mock and a digital-pin mock, constructs the AlertTmp108, asserts
the resolved I2C address, and tears down both mocks."
```

---

## Task 6: Write `examples/continuous.rs`

**Files:**
- Create: `examples/continuous.rs`

- [ ] **Step 1: Write the file**

Create `examples/continuous.rs`:

```rust
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

    tmp.continuous(|t| async {
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
```

**Note** on `delay` capture: `Tmp108::continuous` takes `FnOnce(&mut Self) -> Fut`.
The closure captures `delay` by reference, which is fine — `delay` outlives
the closure call. If borrow-checker complains about move-semantics across the
async boundary, restructure as `move |t| async move { … }` and pass `&mut
delay` via a helper. **Verify in Step 2.**

- [ ] **Step 2: Build and run**

Run: `cargo build --example continuous -F async`
Expected: builds clean.

Run: `cargo run --example continuous -F async`
Expected: prints five lines like `Temperature: 27.19 C` (the value will vary
with ambient temperature). Exits 0.

If a borrow-checker error occurs, restructure as described in Step 1.

- [ ] **Step 3: Confirm the example also builds with no features (inert main)**

Run: `cargo build --example continuous`
Expected: builds clean. Running it should print
`examples/continuous.rs requires --features async` and exit.

- [ ] **Step 4: Commit**

```bash
git add examples/continuous.rs
git commit -m "examples: add continuous-mode example (async-only)

Demonstrates Tmp108::continuous with wait_for_temperature in a loop.
Async-only because Tmp108::continuous is gated on feature = 'async'.
Inert main when async is off so 'cargo build --examples' succeeds in
every feature combination."
```

---

## Task 7: Write `examples/alert_interrupt.rs`

**Files:**
- Create: `examples/alert_interrupt.rs`

- [ ] **Step 1: Write the file**

Create `examples/alert_interrupt.rs`:

```rust
//! TMP108 ALERT pin example — interrupt mode.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//! - TMP108 ALERT pin tied to Pico de Gallo GPIO0 with an external 2 kΩ
//!   pull-up to V+
//!
//! # Cargo features
//!
//! Requires `--features async,embedded-sensors-hal-async`. The `AlertTmp108`
//! wrapper that exposes `wait_for_temperature_threshold` only exists under
//! this combination.
//!
//! # Behavior
//!
//! In interrupt mode the ALERT pin pulses (it clears as soon as the
//! configuration register is read). A `wait_for_temperature_threshold` call
//! in interrupt mode returns once, after which the pin is reset; the next
//! call will block until a new threshold crossing occurs.
//!
//! # Usage
//!
//! Run the example, then warm the TMP108 with a finger (or breathe on it) to
//! cross the 30 °C high threshold. The program will print the temperature at
//! the moment of the trigger and exit.

#[cfg(not(all(feature = "async", feature = "embedded-sensors-hal-async")))]
fn main() {
    eprintln!(
        "examples/alert_interrupt.rs requires --features async,embedded-sensors-hal-async"
    );
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};
    use pico_de_gallo_hal::Hal;
    use pico_de_gallo_lib::{GpioDirection, GpioPull};
    use tmp108::{AlertTmp108, Config, Polarity, Thermostat};

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut alert = hal.gpio(0);
    alert
        .set_config(GpioDirection::Input, GpioPull::None)
        .map_err(|_| anyhow!("Failed to configure GPIO0 as input"))?;

    // README-SNIPPET-START: alert
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
    // README-SNIPPET-END: alert

    Ok(())
}
```

- [ ] **Step 2: Build**

Run: `cargo build --example alert_interrupt -F async,embedded-sensors-hal-async`
Expected: builds clean.

Run: `cargo build --example alert_interrupt`
Expected: builds clean (inert main).

- [ ] **Step 3: Run on hardware**

Run: `cargo run --example alert_interrupt -F async,embedded-sensors-hal-async`
Expected:
1. Prints `Waiting for ALERT (warm the sensor above 30 C)...`
2. Waits. When you warm the TMP108 (finger touch is usually enough), it prints
   `ALERT! Temperature at trigger: <num> C` where `<num>` is ≥ 30.0.
3. Exits 0.

**If it does not trigger:** check that GPIO0 is actually wired to TMP108
ALERT and that the 2 kΩ pull-up is to V+. The ALERT polarity is configured as
ActiveLow, so the falling edge is what we wait for.

- [ ] **Step 4: Commit**

```bash
git add examples/alert_interrupt.rs
git commit -m "examples: add alert_interrupt example

AlertTmp108 in interrupt mode (ALERT polarity active-low). Wires
TMP108 ALERT to Pico de Gallo GPIO0 with an external 2 kohm pull-up.
Trips when temperature exceeds 30 C; prints the temperature at the
trigger and exits. Inert when the required features are off."
```

---

## Task 8: Write `examples/alert_comparator.rs`

**Files:**
- Create: `examples/alert_comparator.rs`

- [ ] **Step 1: Write the file**

Create `examples/alert_comparator.rs`:

```rust
//! TMP108 ALERT pin example — comparator mode (latched behavior).
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//! - TMP108 ALERT pin tied to Pico de Gallo GPIO0 with an external 2 kΩ
//!   pull-up to V+
//!
//! # Cargo features
//!
//! Requires `--features async,embedded-sensors-hal-async`.
//!
//! # Behavior
//!
//! In comparator mode the ALERT pin stays asserted until the temperature
//! returns to within `(T_low + HYS, T_high − HYS)`. This example demonstrates
//! that:
//! - A `wait_for_temperature_threshold` call returns when the threshold is
//!   first crossed.
//! - A *second* call returns *immediately* while the pin is still asserted.
//! - Once the temperature falls back inside the hysteresis band, the pin
//!   releases; the next call blocks again.
//!
//! # Usage
//!
//! Run the example, then warm the TMP108 with a finger to trip the high
//! threshold. Hold the warmth while the program prints two back-to-back
//! readings demonstrating the latched pin, then let it cool below 28 C
//! (high threshold 30 − 2 °C hysteresis) and observe the next call blocks.

#[cfg(not(all(feature = "async", feature = "embedded-sensors-hal-async")))]
fn main() {
    eprintln!(
        "examples/alert_comparator.rs requires --features async,embedded-sensors-hal-async"
    );
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};
    use pico_de_gallo_hal::Hal;
    use pico_de_gallo_lib::{GpioDirection, GpioPull};
    use tmp108::{AlertTmp108, Config, Hysteresis, Polarity, Thermostat};

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut alert = hal.gpio(0);
    alert
        .set_config(GpioDirection::Input, GpioPull::None)
        .map_err(|_| anyhow!("Failed to configure GPIO0 as input"))?;

    let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);

    tmp.tmp108
        .configure(Config {
            thermostat_mode: Thermostat::Comparator,
            alert_polarity: Polarity::ActiveLow,
            hysteresis: Hysteresis::_2C,
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

    println!("Waiting for first ALERT (warm the sensor above 30 C)...");
    let t1 = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("First wait failed"))?;
    println!("First trigger: {t1:.2} C (pin is latched)");

    // While the temperature is still above 30 - 2 = 28 C, this returns
    // immediately because the pin remains asserted.
    let t2 = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("Second wait failed"))?;
    println!("Second trigger (immediate): {t2:.2} C");

    println!("Let the sensor cool below 28 C; the next ALERT will reassert when temperature rises again...");
    let t3 = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("Third wait failed"))?;
    println!("Third trigger: {t3:.2} C");

    Ok(())
}
```

- [ ] **Step 2: Build**

Run: `cargo build --example alert_comparator -F async,embedded-sensors-hal-async`
Expected: builds clean.

Run: `cargo build --example alert_comparator`
Expected: builds clean (inert main).

- [ ] **Step 3: Run on hardware**

Run: `cargo run --example alert_comparator -F async,embedded-sensors-hal-async`
Expected:
1. Prints the "Waiting for first ALERT" message.
2. When you warm the TMP108 above 30 °C: prints `First trigger: <num> C (pin is latched)`.
3. Almost immediately prints `Second trigger (immediate): <num> C` (because the
   pin is still asserted).
4. Prints the "Let the sensor cool" message.
5. Wait for the sensor to cool below 28 °C, then warm it again.
6. Prints `Third trigger: <num> C` and exits 0.

This sequence directly evidences the comparator-mode behavior documented at
`src/lib.rs:602-640`.

- [ ] **Step 4: Commit**

```bash
git add examples/alert_comparator.rs
git commit -m "examples: add alert_comparator example

Demonstrates the latched ALERT pin in comparator mode: two back-to-back
wait_for_temperature_threshold calls both return while the pin is
asserted; the third call blocks until the temperature falls below
T_high - hysteresis and crosses back above it. Inert when the required
features are off."
```

---

## Task 9: Write `examples/sensor_trait.rs`

**Files:**
- Create: `examples/sensor_trait.rs`

- [ ] **Step 1: Write the file**

Create `examples/sensor_trait.rs`:

```rust
//! TMP108 used through the `embedded-sensors-hal` traits.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//!
//! # Cargo features
//!
//! - Blocking variant: `--features embedded-sensors-hal`.
//! - Async variant: `--features async,embedded-sensors-hal-async`.
//!
//! Demonstrates that downstream firmware can call `Tmp108` through the
//! generic `TemperatureSensor` trait without naming the concrete sensor type.
//! This is the pattern to follow when your firmware needs to be parametric
//! over a set of temperature sensors.

#[cfg(all(not(feature = "async"), feature = "embedded-sensors-hal"))]
fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal::temperature::{DegreesCelsius, TemperatureSensor};
    use pico_de_gallo_hal::Hal;
    use tmp108::Tmp108;

    fn read_any_sensor<S: TemperatureSensor>(sensor: &mut S) -> Result<DegreesCelsius, S::Error> {
        sensor.temperature()
    }

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut tmp = Tmp108::new_with_a0_gnd(i2c);

    let temperature = read_any_sensor(&mut tmp).map_err(|_| anyhow!("read failed"))?;
    println!("Temperature (via trait): {temperature:.2} C");
    Ok(())
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{DegreesCelsius, TemperatureSensor};
    use pico_de_gallo_hal::Hal;
    use tmp108::Tmp108;

    async fn read_any_sensor<S: TemperatureSensor>(sensor: &mut S) -> Result<DegreesCelsius, S::Error> {
        sensor.temperature().await
    }

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut tmp = Tmp108::new_with_a0_gnd(i2c);

    let temperature = read_any_sensor(&mut tmp).await.map_err(|_| anyhow!("read failed"))?;
    println!("Temperature (via trait): {temperature:.2} C");
    Ok(())
}

#[cfg(not(any(
    all(not(feature = "async"), feature = "embedded-sensors-hal"),
    all(feature = "async", feature = "embedded-sensors-hal-async"),
)))]
fn main() {
    eprintln!(
        "examples/sensor_trait.rs requires either --features embedded-sensors-hal \
         or --features async,embedded-sensors-hal-async"
    );
}
```

- [ ] **Step 2: Build all relevant feature combinations**

```
cargo build --example sensor_trait
cargo build --example sensor_trait -F embedded-sensors-hal
cargo build --example sensor_trait -F async,embedded-sensors-hal-async
```
Expected: all build clean.

- [ ] **Step 3: Run on hardware**

```
cargo run --example sensor_trait -F embedded-sensors-hal
cargo run --example sensor_trait -F async,embedded-sensors-hal-async
```
Expected: each prints `Temperature (via trait): <num> C` and exits 0.

- [ ] **Step 4: Commit**

```bash
git add examples/sensor_trait.rs
git commit -m "examples: add sensor_trait example

Calls Tmp108 through the embedded-sensors-hal TemperatureSensor trait
(generic free function). Both blocking and async variants. This is the
pattern for firmware that abstracts over multiple temperature-sensor
types."
```

---

## Task 10: Rework `examples/oneshot.rs` to match the new style

**Files:**
- Modify: `examples/oneshot.rs`

- [ ] **Step 1: Replace with the new file**

Replace `examples/oneshot.rs` with:

```rust
//! TMP108 one-shot conversion example.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//!
//! # Cargo features
//!
//! Works with default features (blocking). Building with `--features async`
//! produces the async variant.
//!
//! # Register interactions
//!
//! Single read of the temperature register at address `0x00`.

use anyhow::{anyhow, Result};
use pico_de_gallo_hal::Hal;
use tmp108::Tmp108;

#[cfg(not(feature = "async"))]
fn main() -> Result<()> {
    // README-SNIPPET-START: oneshot
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");
    // README-SNIPPET-END: oneshot

    Ok(())
}

#[cfg(feature = "async")]
#[tokio::main]
async fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp
        .temperature()
        .await
        .map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");

    Ok(())
}
```

- [ ] **Step 2: Build both variants**

```
cargo build --example oneshot
cargo build --example oneshot -F async
```
Expected: both build clean.

- [ ] **Step 3: Run both variants**

```
cargo run --example oneshot
cargo run --example oneshot -F async
```
Expected: each prints `Temperature: <num> C` and exits 0.

- [ ] **Step 4: Commit**

```bash
git add examples/oneshot.rs
git commit -m "examples: rework oneshot to the new doc + snippet style

Bring the existing example into the new format: //! block declaring
hardware, features, and register interactions; README-SNIPPET marker
region around the blocking variant for the README snippet check."
```

---

## Task 11: Write `scripts/check-readme-snippets.sh`

**Files:**
- Create: `scripts/check-readme-snippets.sh`

- [ ] **Step 1: Write the script**

Create `scripts/check-readme-snippets.sh` (file mode 0755):

```bash
#!/usr/bin/env bash
# Verify that the fenced code blocks in README.md tagged with `rust,snippet=NAME`
# match the corresponding README-SNIPPET-START/END regions in examples/.
#
# Usage:
#   scripts/check-readme-snippets.sh
# Exit code: 0 if every snippet matches, 1 if any drift is detected.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
README="$REPO_ROOT/README.md"
EXAMPLES_DIR="$REPO_ROOT/examples"

fail=0

# List of snippet name -> example file (extend when new snippets are added).
declare -A SNIPPETS=(
    [oneshot]="oneshot.rs"
    [continuous]="continuous.rs"
    [alert]="alert_interrupt.rs"
)

extract_from_example() {
    local name="$1"
    local file="$2"
    awk -v marker="$name" '
        $0 ~ "README-SNIPPET-START: "marker"$" { capture=1; next }
        $0 ~ "README-SNIPPET-END: "marker"$"   { capture=0 }
        capture { sub(/^[[:space:]]{4}/, "", $0); print }
    ' "$file"
}

extract_from_readme() {
    local name="$1"
    awk -v marker="$name" '
        $0 ~ "^```rust,snippet="marker"$" { capture=1; next }
        capture && /^```$/                { capture=0 }
        capture { print }
    ' "$README"
}

for name in "${!SNIPPETS[@]}"; do
    file="$EXAMPLES_DIR/${SNIPPETS[$name]}"
    if [[ ! -f "$file" ]]; then
        echo "snippet $name: missing example file $file" >&2
        fail=1
        continue
    fi
    expected=$(extract_from_example "$name" "$file")
    actual=$(extract_from_readme "$name")
    if [[ -z "$expected" ]]; then
        echo "snippet $name: no START/END markers found in $file" >&2
        fail=1
        continue
    fi
    if [[ -z "$actual" ]]; then
        echo "snippet $name: no fenced block ```rust,snippet=$name``` in README.md" >&2
        fail=1
        continue
    fi
    if [[ "$expected" != "$actual" ]]; then
        echo "snippet $name: drift detected between $file and README.md" >&2
        diff <(echo "$expected") <(echo "$actual") | sed 's/^/    /' >&2
        fail=1
    fi
done

if [[ $fail -ne 0 ]]; then
    echo "README snippets are out of sync. Re-copy from examples/ into README.md." >&2
    exit 1
fi
echo "README snippets match (${#SNIPPETS[@]} checked)."
```

The 4-space `sub()` strips the indentation each snippet has inside its
`async fn main` (the README presents top-level code).

- [ ] **Step 2: Make it executable and run it (will fail until README is updated)**

```
chmod +x scripts/check-readme-snippets.sh
scripts/check-readme-snippets.sh
```
Expected, before Task 12: fails with `snippet …: no fenced block …` for at
least the new snippets. This is correct — it shows the script can detect
drift. Don't commit yet; the README update in Task 12 must land before the
script returns 0.

- [ ] **Step 3: Do NOT commit yet.** The script depends on Task 12 to pass.

---

## Task 12: Rewrite `README.md`

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Replace the file**

Overwrite `README.md` with:

````markdown
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

```rust,snippet=oneshot
let hal = Hal::new();
let i2c = hal.i2c();

let mut tmp = Tmp108::new_with_a0_gnd(i2c);
let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
println!("Temperature: {temperature:.2} C");
```

### Async continuous conversions

```rust,snippet=continuous
let hal = Hal::new();
let i2c = hal.i2c();
let mut delay = hal.delay();

let mut tmp = Tmp108::new_with_a0_gnd(i2c);

tmp.continuous(|t| async {
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

```rust,snippet=alert
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
````

- [ ] **Step 2: Run the snippet check**

Run: `scripts/check-readme-snippets.sh`
Expected: `README snippets match (3 checked).`

If it fails for a snippet, copy the exact contents from
`extract_from_example "<name>" "examples/<file>"` into the README's fenced
block. Common pitfalls: extra leading whitespace, trailing whitespace, or
forgetting that the script strips four spaces of indentation.

- [ ] **Step 3: Confirm rustdoc still builds (README is included via `#![doc = include_str!]`)**

Run: `cargo doc --no-deps --all-features`
Expected: builds clean. The fenced blocks tagged `rust,snippet=NAME` are
treated as plain (non-test) by rustdoc — the unknown info string after `rust`
is harmless.

If rustdoc complains, change `rust,snippet=NAME` to `text` in the README and
update `extract_from_readme` in the script to match `^```text$` with a
following `# snippet: NAME` line.

- [ ] **Step 4: Commit**

```bash
git add README.md scripts/check-readme-snippets.sh
git commit -m "docs: rewrite README with CI-verified snippets

Three usage snippets (oneshot, continuous, alert) are now lifted
verbatim from examples/oneshot.rs, examples/continuous.rs, and
examples/alert_interrupt.rs via README-SNIPPET marker regions.
scripts/check-readme-snippets.sh enforces byte-equality and is wired
into CI in the next task.

Adds a feature-flag table and a 'gotchas' list addressing the most
common footguns: no delay in Tmp108::new, comparator vs interrupt
latching, ALERT polarity wiring, async-only continuous, temperature
scale."
```

---

## Task 13: Document the snippet marker convention in `CONTRIBUTING.md`

**Files:**
- Modify: `CONTRIBUTING.md`

- [ ] **Step 1: Append a section**

Append to `CONTRIBUTING.md`:

```markdown

## README snippet markers

The "Usage" section of `README.md` contains snippets lifted verbatim from
runnable example programs in `examples/`. To enforce that the README cannot
drift from the API:

1. In an example file, wrap the snippet body in markers:

   ```rust
   // README-SNIPPET-START: <name>
   <code>
   // README-SNIPPET-END: <name>
   ```

   The code between the markers is stripped of four spaces of indentation
   when compared (so it can live inside an `async fn main`).

2. In `README.md`, present the same code in a fenced block tagged
   `rust,snippet=<name>`.

3. Register the `<name> -> file.rs` mapping in `scripts/check-readme-snippets.sh`'s
   `SNIPPETS` map.

CI runs `scripts/check-readme-snippets.sh`. If a snippet drifts, fix the
README to match the example (the example is the source of truth — it
compiles, runs, and is exercised on real hardware).
```

- [ ] **Step 2: Commit**

```bash
git add CONTRIBUTING.md
git commit -m "docs: document README snippet marker convention

Adds a 'README snippet markers' section to CONTRIBUTING.md pointing
contributors at the marker convention and the snippet check script."
```

---

## Task 14: Extend CI to run doctests, build all examples, and check README snippets

**Files:**
- Modify: `.github/workflows/check.yml`

- [ ] **Step 1: Extend the `test` job**

Edit `.github/workflows/check.yml`. After the existing `cargo test
(embedded-sensors-hal)` step (around line 200), append:

```yaml
      - name: cargo test --doc (blocking)
        run: cargo test --doc --locked

      - name: cargo test --doc (async)
        run: cargo test --doc --locked -F async

      - name: cargo test --doc (embedded-sensors-hal-async)
        run: cargo test --doc --locked -F async,embedded-sensors-hal-async

      - name: cargo build --examples (default)
        run: cargo build --examples --locked

      - name: cargo build --examples (async)
        run: cargo build --examples --locked -F async

      - name: cargo build --examples (embedded-sensors-hal)
        run: cargo build --examples --locked -F embedded-sensors-hal

      - name: cargo build --examples (embedded-sensors-hal-async)
        run: cargo build --examples --locked -F async,embedded-sensors-hal-async

      - name: README snippets in sync
        run: ./scripts/check-readme-snippets.sh
```

- [ ] **Step 2: Validate the file locally**

If `actionlint` is available: `actionlint .github/workflows/check.yml`.
Otherwise, eyeball the YAML for matching indentation against the surrounding
steps.

- [ ] **Step 3: Run the new checks locally to verify they pass**

Sequentially:
```
cargo test --doc --locked
cargo test --doc --locked -F async
cargo test --doc --locked -F async,embedded-sensors-hal-async
cargo build --examples --locked
cargo build --examples --locked -F async
cargo build --examples --locked -F embedded-sensors-hal
cargo build --examples --locked -F async,embedded-sensors-hal-async
./scripts/check-readme-snippets.sh
```
Expected: all green.

- [ ] **Step 4: Commit**

```bash
git add .github/workflows/check.yml
git commit -m "ci: run doctests, build all examples, check README snippets

Extends the test job with:
- cargo test --doc for blocking, async, and embedded-sensors-hal-async
- cargo build --examples for each supported feature combination
- scripts/check-readme-snippets.sh to enforce README/examples parity"
```

---

## Task 15: Final verification pass

- [ ] **Step 1: Run the full local CI matrix**

Sequentially, from the repo root:
```
cargo fmt --check
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
cargo doc --no-deps --all-features --locked
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
cargo test --doc --locked
cargo test --doc --locked -F async
cargo test --doc --locked -F async,embedded-sensors-hal-async
cargo build --examples --locked
cargo build --examples --locked -F async
cargo build --examples --locked -F embedded-sensors-hal
cargo build --examples --locked -F async,embedded-sensors-hal-async
./scripts/check-readme-snippets.sh
```
Expected: every command exits 0.

- [ ] **Step 2: Run every example on the attached Pico de Gallo**

```
cargo run --example oneshot
cargo run --example oneshot -F async
cargo run --example continuous -F async
cargo run --example sensor_trait -F embedded-sensors-hal
cargo run --example sensor_trait -F async,embedded-sensors-hal-async
cargo run --example alert_interrupt -F async,embedded-sensors-hal-async   # warm the sensor when prompted
cargo run --example alert_comparator -F async,embedded-sensors-hal-async  # warm, then cool, then warm
```
Expected: each prints sensible output and exits 0. The alert examples require
human interaction at the prompts.

- [ ] **Step 3: Confirm rustdoc renders the new snippets correctly**

Run: `cargo doc --no-deps --all-features --open`
Spot-check: `Tmp108` page lists every method from §3.2 of the spec, each with
an "Examples" subsection that compiles.

- [ ] **Step 4: Push the branch**

(If on a working branch, push and open a draft PR per `CONTRIBUTING.md`. If on
`main`, skip — the user will handle integration.)

---

## Spec coverage check (self-review, post-write)

| Spec section | Tasks covering it |
|--------------|-------------------|
| §3.0 Re-exports | Task 1 |
| §3.1 README rewrite | Task 12 |
| §3.2 Per-method doctests on Tmp108 | Tasks 2, 3, 4 |
| §3.2 Per-method doctests on AlertTmp108 | Task 5 |
| §3.3 examples/oneshot.rs | Task 10 |
| §3.3 examples/continuous.rs | Task 6 |
| §3.3 examples/alert_interrupt.rs | Task 7 |
| §3.3 examples/alert_comparator.rs | Task 8 |
| §3.3 examples/sensor_trait.rs | Task 9 |
| §3.3 Inert-when-features-off invariant | Tasks 6, 7, 8, 9 (Step 3 of each) |
| §3.4 Snippet extraction script | Task 11 |
| §3.4 CONTRIBUTING.md convention | Task 13 |
| §3.5 CI changes | Task 14 |
| §6 Acceptance criteria | Task 15 (final verification) |

All spec sections accounted for.
