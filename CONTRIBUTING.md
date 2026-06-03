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

* Use meaningful commit messages. See [this blogpost](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html)

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
