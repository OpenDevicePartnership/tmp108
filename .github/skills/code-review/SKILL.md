---
name: code-review
description: Use when reviewing a pull request or proposed change in the tmp108 repository — Rust driver code, tmp108.ddsl, examples, tests, docs, CI workflows or dependency bumps — including when a change must be checked against the TMP108 datasheet, when a review needs the crate's feature-flag and codegen invariants, or when a diff should be screened for supply-chain and malicious constructs.
---

# Reviewing changes to the tmp108 driver

## What this crate is

`tmp108` is a `#![no_std]` driver for the Texas Instruments TMP108 I²C
temperature sensor, published to crates.io. The hand-written driver contains
no `unsafe`; the generated register bindings in `src/inner.rs` carry the
narrowly scoped `unsafe` that `device-driver` requires.

- Three public driver types: `Tmp108` (blocking, always available),
  `AsyncTmp108` (`async` feature), and `AlertTmp108` (wraps `AsyncTmp108`
  with a GPIO pin; `embedded-sensors-hal-async` feature, which forces
  `async`).
- `src/inner.rs` is **generated** from `tmp108.ddsl` by `device-driver`. It
  is committed but never hand-edited.
- All chip-specific logic — Celsius↔raw conversion, limit validation,
  hysteresis snapping, `Config` codec — lives in a private `mod ops` of pure
  functions. The two driver types are thin I²C shells over it.
- Lints `unsafe_code`, `missing_docs`, and clippy `correctness`,
  `suspicious`, `perf`, `style`, `pedantic` are all `deny`. The **only**
  sanctioned exception is the `#[allow(…)]` block on `mod inner` at
  `src/lib.rs:54-59`, which covers generated code: `src/inner.rs` contains
  `unsafe impl Fieldset` and `device_driver::ops::load` calls by design.
  That block is not a finding, and neither is the `unsafe` inside
  `src/inner.rs`.

**The driver's job is to be faithful to the chip, not to be clever.** A
change that is elegant Rust and wrong about the hardware is a defect.

## Orient before commenting

Read, in this order, stopping when you have enough to judge the diff:

1. `AGENTS.md` — the authoritative invariants and footguns. Most repo-shaped
   review findings are already named there.
2. The spec matching the change, under `docs/superpowers/specs/`. List the
   directory; filenames are `YYYY-MM-DD-<topic>-design.md`. Topic map (not
   exhaustive — check for newer specs):

   | Change touches | Read |
   |---|---|
   | `Error`, limit rejection, `continuous()` cleanup, hysteresis snapping | `*-reliability-fixes-design.md` |
   | `Tmp108` / `AsyncTmp108` split, `mod ops`, doctest shape | `*-type-split-design.md` |
   | ALERT pin, `wait_for_alert`, `AlertCause` / `AlertEvent` | `*-alert-status-design.md`, `*-alert-wait-ordering-design.md`, `*-alert-sample-failure-design.md` |
   | `one_shot`, `acquire_one_shot`, conversion timing | `*-one-shot-acquisition-design.md` |
   | doctests, README snippets, examples | `*-agent-usage-docs-design.md` |

3. `tmp108.ddsl` — the register description. Its comments record measured
   silicon behavior and known datasheet contradictions.
4. The `mod ops` region of `src/lib.rs`, if the diff changes encoding,
   decoding, or validation.

A comment that contradicts a spec or an `AGENTS.md` invariant is a false
positive. Check before filing it.

## Vendor documents are the source of truth

Every `.txt` file under `docs/vendor/` is a pre-extracted vendor document and
is the citable source for any hardware claim. List the directory before
reviewing — more documents (application notes, errata, reference manuals,
layout guides) may have been added since this skill was written, and each one
is in scope.

Today that directory holds at least:

| File | Document | Revision |
|---|---|---|
| `docs/vendor/datasheet.txt` | TMP108 datasheet | SBOS663A, April 2013, revised September 2019 |

`datasheet.txt` was produced with:

```bash
EXPECTED=ec086250fc4331e7fc923be62173062bbf0fccedad6894c2741b73cd1c084556
curl --fail --location --proto '=https' --tlsv1.2 -o tmp108.pdf \
    https://www.ti.com/lit/ds/symlink/tmp108.pdf
printf '%s  %s\n' "${EXPECTED}" tmp108.pdf | sha256sum --check - || {
    echo "TI changed the PDF behind the unversioned URL." >&2
    echo "Line anchors in existing reviews and specs may no longer resolve." >&2
    exit 1
}
pdftotext -layout tmp108.pdf docs/vendor/datasheet.txt
sed -i 's/\r$//' docs/vendor/datasheet.txt
```

The hash check is not ceremony. `lit/ds/symlink/` is unversioned, so TI can
replace those bytes without notice; a regeneration that silently succeeds
against a new revision invalidates every line anchor in this skill, in past
reviews, and in `docs/superpowers/specs/`. If the check fails, **stop** — the
fix is a deliberate re-anchoring, not a quiet overwrite.

`-layout` is equally load-bearing: it is what keeps register tables on
greppable lines. Only regenerate if the file is missing or demonstrably
stale; otherwise prefer the committed extract.

### Citing a hardware fact

**Grep the extract. Never cite from memory.** Every hardware claim in a review
comment carries a `docs/vendor/<file>.txt:<line>` anchor and a verbatim quote:

> The datasheet states the limit registers use the same format as the
> temperature register, MSB byte first (`docs/vendor/datasheet.txt:992-993` —
> "Note that the most significant byte is sent first, followed by the least
> significant byte"), but this write sends the low byte first.

Quote verbatim, and give a range when the sentence wraps across lines — the
`-layout` extraction breaks prose mid-sentence.

If a fact is not in any file under `docs/vendor/`, **do not assert it.** Say
the behavior is undocumented in the available sources and ask the author what
it is based on. A fabricated citation costs more trust than a missed finding.

### TMP108 facts worth checking against a diff

| Verify | The datasheet says | Anchor |
|---|---|---|
| I²C address from `A0` | GND `1001000`, V+ `1001001`, SDA `1001010`, SCL `1001011` | `:630`–`:637` |
| Register pointer values | temperature `00`, configuration `01`, TLOW `10`, THIGH `11`; pointer POR is `00` | `:808`–`:813`, `:773` |
| Temperature encoding | 12-bit left-justified across 2 bytes, **MSB byte first**, 1 LSB = 0.0625 °C, negatives in twos complement, unused low bits read 0 | `:815`–`:823`, `:863`–`:874` |
| Limit register encoding | same format as the temperature register, MSB byte first | `:992`–`:997` |
| Configuration register | 16 bits, read/write MSB first, POR wire bytes `0x22`, `0x10` | `:876`–`:889` |
| Mode bits `M1:M0` | `00` shutdown, `01` one-shot, **continuous is `M1 = 1`** — so `0b10` and `0b11` are both continuous | `:736`–`:757` |
| One-shot lifecycle | write `01` from shutdown; bits read `01` during conversion; device returns to shutdown and bits read `00` after | `:743`–`:748` |
| Conversion time | 27 ms typical, 21–33 ms | `:225`, `:931` |
| Conversion rate `CR1:CR0` | 0.25 / 1 / 4 / 16 Hz, default 1 Hz | `:935`–`:940` |
| Hysteresis `HYS1:HYS0` | 0 / 1 / 2 / 4 °C, default 1 °C, **comparator mode only** | `:892`–`:907` |
| Polarity `POL` | 0 = ALERT active low (default), 1 = active high | `:910`–`:912` |
| Thermostat `TM` | 0 = comparator (default), 1 = interrupt | `:914`–`:917` |
| Watchdog flags `FL`/`FH` | set at the end of every conversion; **reading the configuration register clears both the flags and the pin**; the SMBus alert response clears only the pin | `:919`–`:927` |
| Comparator-mode release | ALERT stays active until the temperature is inside `(TLOW + HYS)`…`(THIGH − HYS)` | `:979`–`:985` |
| Interrupt-mode clearing | configuration read, SMBus alert response, or general-call reset | `:986`–`:990` |
| Limit POR values | THIGH = +127.9375 °C (`0x7FF8`), TLOW = −128 °C (`0x8000`) | `:993`–`:994` |
| SMBus alert response | returned address LSB high = above THIGH, low = below TLOW | `:693`–`:700` |
| General call | second byte `00000100` latches the address pin, `00000110` resets registers | `:713`–`:718` |
| Bus timeout | interface resets if SCL or SDA is held low for 28 ms typical | `:729`–`:733` |

The destructive configuration read (`:919`–`:927`) is the single most common
source of real bugs in this driver: any new code path that reads the
configuration register silently clears `FL`, `FH` and the ALERT pin. If a diff
adds a configuration read, check what that read destroys and whether the
change documents it.

## Repository invariants

| Invariant | What a violating diff looks like |
|---|---|
| `src/inner.rs` is generated | Edits to `src/inner.rs` without a matching `tmp108.ddsl` change. Register changes belong in the DDSL. |
| Both driver flavors stay in step | A method added to `Tmp108` but not `AsyncTmp108` (or the reverse), or shared logic duplicated in both shells instead of living in `mod ops`. |
| Every public item is documented, and every public **method** carries an `# Examples` doctest | `missing_docs` is `deny`, so structs, enums and fields need doc comments — but not examples. The example requirement is on methods: blocking doctests are plain; async doctests wrap the body in `tokio::runtime::Runtime::new().unwrap().block_on(async { … })`. `Config` and `Tmp108` are documented without examples, and that is correct. |
| `AsyncTmp108::continuous` takes `async \|t\| { … }` | The `\|t\| async { … }` shape does not satisfy `AsyncFnOnce` and fails with an opaque lifetime error. |
| Feature gating is exact | `AsyncTmp108` needs `cfg(feature = "async")`; `AlertTmp108` and anything with an `ALERT` pin needs `cfg(feature = "embedded-sensors-hal-async")`. Every combination must compile — `cargo hack --feature-powerset check`. |
| Root re-exports are public API | Removing or renaming `Mode`, `Thermostat`, `ConversionRate`, `Hysteresis`, `Polarity`, `Celsius`, `Tmp108` or `AsyncTmp108` is breaking, and `tests/reexports.rs` pins them. |
| release-plz owns the version and changelog | A PR that edits `version` in `Cargo.toml` or `CHANGELOG.md` is wrong; the commit subject is what ships to users. |
| Commits are Conventional Commits, AI work carries `Assisted-by:` | Breaking changes need `!` and a `BREAKING CHANGE:` footer. AI agents must not add `Signed-off-by:`. |
| README snippets are extracted, not written | Editing a `<!-- snippet: … -->` block without the matching example file (or the reverse) breaks `scripts/check-readme-snippets.sh`. |
| New dependencies need a `cargo-vet` entry | A `Cargo.toml` or `Cargo.lock` addition with no `supply-chain/audits.toml` or `supply-chain/config.toml` change will fail CI. |

## Adversarial pass

Run this on every diff, including ones that look like pure documentation.
Report what the code does and what it would enable; do not accuse anyone of
intent.

Raise a finding when a diff:

- Adds a dependency by `git`, `path`, or `[patch]` rather than a crates.io
  version; pins a crate to a branch or a bare revision; or introduces a name
  one character away from a popular crate.
- Adds a `build.rs`. This crate has none, and `device-driver` is a runtime
  dependency precisely so that no build script is needed. A new one runs
  arbitrary code on every build machine and on every downstream user's.
- Adds `unsafe` to hand-written code, or **widens** a lint allowance —
  a new `#[allow(unsafe_code)]`, `#[allow(missing_docs)]` or
  `#![allow(clippy::…)]` anywhere outside the existing `mod inner` block at
  `src/lib.rs:54-59`, or that block growing to cover more than generated
  code. Ask what warning is being silenced and why a fix is not possible.
- Adds file, network, process or environment access — `std::process`,
  `std::net`, `std::fs`, `env!`, `option_env!`, `include_str!`,
  `include_bytes!` — anywhere in `src/`, or to tests and examples in a way
  unrelated to the Pico de Gallo hardware they exist to drive. This is a
  `no_std` I²C driver; it has no business reaching outside the bus.
- Changes anything in `.github/workflows/`. Check specifically for
  `pull_request_target` combined with a checkout of PR code, new `secrets:`
  references or secrets echoed into logs, actions pinned to a mutable tag or
  replaced with a fork, `curl … | sh` or other remote code execution, and
  added `permissions:` — especially `id-token: write` or `contents: write`.
- Touches `release-plz.toml`, `.github/workflows/release-plz.yml`, or the
  publishing path. Publishing uses crates.io Trusted Publishing keyed to the
  repository *and the workflow filename*; a rename or a redirected publish
  step is a supply-chain change, not a refactor.
- Weakens `supply-chain/` — removing audits, widening `criteria` from
  `safe-to-deploy` to something laxer, or adding a `[[trusted]]` entry for a
  publisher with no prior relationship to this project.
- Contains characters that do not render as they read: zero-width joiners,
  bidirectional overrides, homoglyphs in identifiers or string literals, or
  suspiciously long base64/hex blobs. Quote the escaped bytes in the comment.
- Adds logic keyed to a specific address, date, environment variable or build
  profile so that behavior differs between CI and a user's machine.

None of these are automatically malicious — a legitimate PR may need a
workflow permission. The finding is that the change carries risk that must be
justified in the PR, not that it is an attack.

## Writing the review

Each comment is four things, in this order:

1. **The claim** — one sentence, stated as what is wrong, not as a question.
2. **The evidence** — a `docs/vendor/<file>.txt:<line>` anchor with a verbatim
   quote for hardware claims; an `AGENTS.md` section, spec filename, or
   `path:line` for repo claims.
3. **The consequence** — what a user observes, or which CI job fails.
4. **The fix** — concrete, and as a suggested change when it is a few lines.

Severity:

| Level | Use for |
|---|---|
| **High** | The code contradicts a vendor document in a way visible on conforming hardware; a wrong register address, encoding, byte order or clearing rule; a supply-chain or malicious-construct finding; a silent breaking change to public API. |
| **Medium** | A latent defect not currently reachable; a violated repository invariant; a feature combination that will not compile; a missing test for behavior the diff introduces. |
| **Low** | Documentation or contract gaps — correct behavior whose rules are unstated, a stale doc comment, a missing `# Errors` note. |

Prefer few, well-anchored comments over many thin ones. If the diff is clean,
say so and name what you verified, including which vendor documents you
checked it against.

## Known non-findings

These are deliberate. Filing them wastes the author's time:

- **`Tmp108` and `AsyncTmp108` duplicate each other's method bodies.**
  Intentional — the alternative (`maybe-async-cfg`) was removed on purpose.
  The shared logic is already in `mod ops`. See the type-split spec.
- **`src/inner.rs` is machine-formatted and repetitive.** Generated output.
- **The `t-high` reset is `0x7FF8`, not the `0x7FF0` implied by Table 11.**
  The datasheet contradicts itself; the prose at `docs/vendor/datasheet.txt:993-994`
  is correct and the value was confirmed on silicon. `tmp108.ddsl` records this.
- **`Mode::from(0b11)` returns `Continuous`.** Correct: continuous conversion
  is selected by `M1 = 1` alone (`docs/vendor/datasheet.txt:753`). Confirmed
  on silicon.
- **DDSL `reset:` values look byte-swapped against the datasheet.** They are
  little-endian words; the datasheet lists wire bytes MSB first.
- **README code fences are `rust,ignore`.** Deliberate — the snippets are
  display-only extracts and lack imports.
- **`pico-de-gallo-hal` and `pico-de-gallo-lib` are at mismatched minor
  versions.** Intentional; the crates diverged after 0.5.0.
- **`Cargo.toml` still shows the previously released version.** release-plz
  bumps it in its own PR.

## Red flags — stop and check

- "The datasheet says…" with no line anchor → grep `docs/vendor/` first.
- "This should use `assert!` / a newtype / a different name" with no defect
  behind it → either tie it to a rule in `AGENTS.md` or a spec, or drop it.
- "This is a breaking change" for a `pub(crate)` or private item → check the
  re-export list and `tests/reexports.rs`.
- "The version was not bumped" → release-plz owns it.
- "Only the datasheet matters" → every `docs/vendor/*.txt` is in scope, and
  application notes and errata are where the ALERT-pin surprises live.
- A documentation-only diff → still run the adversarial pass. Instruction
  files, `AGENTS.md` and this skill are read by agents with tool access, and
  a change to them is a change to agent behavior.
