#!/usr/bin/env bash
# Creates an isolated feature worktree so agents do not edit the release tree.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
WORKTREE_PARENT="$(dirname "$ROOT")/.agent-worktrees"

usage() {
    echo "Usage: jcy_agent_worktree.sh create <agent>-<short-task>" >&2
}

if [[ "${1:-}" != "create" || -z "${2:-}" || $# -ne 2 ]]; then
    usage
    exit 64
fi

slug="$2"
if [[ ! "$slug" =~ ^[a-z0-9][a-z0-9-]{2,63}$ ]]; then
    echo "Use a lowercase slug: <agent>-<short-task>" >&2
    exit 64
fi

# Runtime captures are intentionally ignored, but any source/config change in
# the canonical tree must be integrated before another worktree starts.
dirty="$(git -C "$ROOT" status --porcelain | \
    grep -vE '^\?\? (backups|diagnostics|screenshots)/' || true)"
if [[ -n "$dirty" ]]; then
    echo "Canonical release tree has unintegrated changes:" >&2
    printf '%s\n' "$dirty" >&2
    echo "Do not create a parallel worktree yet; ask its owner to commit first." >&2
    exit 2
fi

branch="codex/$slug"
target="$WORKTREE_PARENT/$slug"
if git -C "$ROOT" show-ref --verify --quiet "refs/heads/$branch"; then
    echo "Branch already exists: $branch" >&2
    exit 3
fi
if [[ -e "$target" ]]; then
    echo "Worktree path already exists: $target" >&2
    exit 3
fi

mkdir -p "$WORKTREE_PARENT"
git -C "$ROOT" worktree add -b "$branch" "$target" HEAD
echo "Created isolated worktree: $target"
echo "Branch: $branch"
