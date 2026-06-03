#!/usr/bin/env bash
# Verify that the fenced code blocks in README.md preceded by `<!-- snippet: NAME -->`
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
        $0 ~ "^<!-- snippet: "marker" -->$" { in_marker=1; next }
        in_marker && /^```rust,ignore$/     { in_marker=0; capture=1; next }
        capture && /^```$/                  { capture=0 }
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
        echo "snippet $name: no '<!-- snippet: $name -->' / fenced rust block in README.md" >&2
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
