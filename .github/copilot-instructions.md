# Copilot Instructions

This file is kept as a stub for backwards compatibility with GitHub
Copilot, which loads `.github/copilot-instructions.md` automatically.

**The authoritative guidance for AI agents working in this repository
lives in [`AGENTS.md`](../AGENTS.md) at the repo root.** Read that file
before making any changes. It covers:

- repository layout and the feature-flag matrix
- the full local CI matrix you must pass before pushing
- numbered gotchas (nightly-only fmt, the `maybe_async_cfg` doctest
  pattern, `Tmp108::continuous`'s required `async |t| { ... }` syntax,
  the `Config` field-type re-exports, etc.)
- Pico de Gallo hardware setup for running the examples
- the Conventional Commits v1.0.0 commit-message convention
- the **mandatory** `Assisted-by:` trailer on every AI-generated commit
  (and the prohibition against `Signed-off-by:` from non-humans)
- where to put new code / tests / docs

If you only read one paragraph: this repository uses Conventional
Commits, requires `cargo +nightly fmt --check` (not stable fmt) to pass
in CI, and requires every AI-assisted commit to end with an
`Assisted-by: AGENT_NAME:MODEL_VERSION` trailer. Verify your own model
identity before composing that trailer — do not hard-code a name from
a previous session.
