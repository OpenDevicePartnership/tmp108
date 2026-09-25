# Vendor documentation

Text extracts of the vendor documents that define the part this crate
drives. They exist so that a code review can cite the specification by
line and quote it verbatim, instead of asserting hardware behaviour from
memory.

## Notice

**The files in this directory are reproductions of third-party vendor
documentation. They are not covered by this repository's MIT license and
remain the copyright of their respective owners.**

They are reproduced here solely to support review of this driver against
the part's specification — principally by automated, AI-based code review
agents, which cannot reliably fetch and extract a PDF during a review, and
by the human reviewers who check their citations. They are not
redistributed as documentation, are not a substitute for the vendor's
official publications, and are excluded from the published crate by the
`include` allowlist in `Cargo.toml`.

**Always consult the vendor's official documents as the authoritative
source.** For the TMP108, that is the product folder at
<https://www.ti.com/product/TMP108> and the datasheet at
<https://www.ti.com/lit/ds/symlink/tmp108.pdf>. TI's terms of use for its
technical documentation are at <https://www.ti.com/legal/terms-conditions/terms-of-use.html>.

Each extract includes the vendor's own notices verbatim; nothing has been
removed. The only transformation is PDF-to-text conversion and newline
normalisation, both reproducible with the commands below.

## Contents

| File | Document | Revision | Source PDF `sha256` |
|---|---|---|---|
| `datasheet.txt` | TMP108 Low Power Digital Temperature Sensor With Two-Wire Serial Interface | SBOS663A — April 2013, revised September 2019 | `ec086250fc4331e7fc923be62173062bbf0fccedad6894c2741b73cd1c084556` |

Application notes, errata, reference manuals and layout guides may be added
here over time. Anything in this directory with a `.txt` extension is
treated as a citable source by the `code-review` agent skill.

## Regenerating an extract

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

Two details are load-bearing:

- **`-layout`** is what keeps register tables and numbered headings on
  greppable lines. Without it the tables collapse and the extract stops
  being citable.
- **The hash check is not ceremony.** TI's `lit/ds/symlink/` URL is
  unversioned, so the bytes behind it can be replaced without notice. A
  regeneration that silently succeeds against a new revision invalidates
  every line anchor in the `code-review` skill, in past review comments,
  and in `docs/superpowers/specs/`. If the check fails, **stop** — the
  response is a deliberate, reviewed re-anchoring, not a quiet overwrite.

Only regenerate when an extract is missing or demonstrably stale.

## How these files are cited

`.github/skills/code-review/SKILL.md` requires every hardware claim in a
review to carry a `docs/vendor/<file>.txt:<line>` anchor and a verbatim
quote, so that any reader can re-check it with `grep`. That is the whole
reason these extracts are committed rather than fetched: line numbers only
mean something if they are stable and shared.
