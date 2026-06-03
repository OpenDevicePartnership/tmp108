# Contributing to Open Device Partnership

The Open Device Partnership project welcomes your suggestions and contributions! Before opening your first issue or pull request, please review our
[Code of Conduct](CODE_OF_CONDUCT.md) to understand how our community interacts in an inclusive and respectful manner.

## Contribution Licensing

Most of our code is distributed under the terms of the [MIT license](LICENSE), and when you contribute code that you wrote to our repositories,
you agree that you are contributing under those same terms. In addition, by submitting your contributions you are indicating that
you have the right to submit those contributions under those terms.

## Other Contribution Information

If you wish to contribute code or documentation authored by others, or using the terms of any other license, please indicate that clearly in your
pull request so that the project team can discuss the situation with you.

## Commit Message

Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
v1.0.0 format. The subject line follows:

```
<type>[optional scope]: <description>
```

**Allowed types** (lowercase, no trailing punctuation in the subject):

- `feat` — a new feature
- `fix` — a bug fix
- `docs` — documentation only
- `style` — formatting, whitespace, missing semicolons (no semantic change)
- `refactor` — code change that neither fixes a bug nor adds a feature
- `perf` — a performance improvement
- `test` — adding or fixing tests
- `build` — build system or external dependencies
- `ci` — CI configuration and scripts
- `chore` — maintenance work that doesn't fit elsewhere
- `revert` — reverts a previous commit

**Subject line rules** (in addition to the type prefix):

- Imperative mood ("add", not "adds" or "added")
- 50 characters or less when feasible, hard limit 72
- No trailing period
- Capitalize only proper nouns; the description is otherwise lowercase

**Body** (optional but encouraged for non-trivial changes):

- Separate subject from body with a blank line
- Wrap at 72 characters
- Explain *what* and *why*, not *how*
- Reference issues/PRs by number when relevant (`Closes #42`, `Refs #17`)

**Breaking changes:** append `!` after the type/scope and include a
`BREAKING CHANGE:` footer that describes the impact and migration path:

```
feat(api)!: replace Tmp108::continuous closure shape

BREAKING CHANGE: continuous now takes `AsyncFnOnce(&mut Self) -> Result<...>`
instead of `FnOnce(&mut Self) -> Fut`. Callers must use `async |t| { ... }`
in place of `|t| async { ... }`.
```

**AI attribution** — see [.github/copilot-instructions.md](.github/copilot-instructions.md)
or [AGENTS.md](AGENTS.md). Every commit produced with AI assistance must end
with an `Assisted-by:` trailer.

**Examples** drawn from this repository's history:

```
fix: config register byte order
docs: add # Examples doctests to Tmp108 threshold methods
build: bump pico-de-gallo-hal dev-dep to 0.5 + add pico-de-gallo-lib
ci: run doctests, build all examples, check README snippets
```

## PR Etiquette

* Create a draft PR first
* Make sure that your branch has `.github` folder and all the code linting/sanity check workflows are passing in your draft PR before sending it out to code reviewers.

## Clean Commit History

We disabled squashing of commit and would like to maintain a clean commit history. So please reorganize your commits with the following items:

* Each commit builds successfully without warning
* Miscellaneous commits to fix typos + formatting are squashed

## Regressions

When reporting a regression, please ensure that you use `git bisect` to find the first offending commit, as that will help us finding the culprit a lot faster.

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

2. In `README.md`, present the same code as an HTML comment followed by an
   `ignore`d Rust fence:

   ````markdown
   <!-- snippet: <name> -->
   ```rust,ignore
   <code>
   ```
   ````

   The `,ignore` keeps rustdoc from trying to compile the snippet (it lacks
   imports and is for display only); the `<!-- snippet: ... -->` HTML
   comment is the marker the checker matches.

3. Register the `<name> -> file.rs` mapping in `scripts/check-readme-snippets.sh`'s
   `SNIPPETS` map.

CI runs `scripts/check-readme-snippets.sh`. If a snippet drifts, fix the
README to match the example (the example is the source of truth — it
compiles, runs, and is exercised on real hardware).
