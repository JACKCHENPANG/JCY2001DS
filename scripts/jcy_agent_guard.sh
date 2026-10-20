#!/usr/bin/env bash
# Atomic local lock for the canonical JCY2001D release worktree.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
LOCK_DIR="$ROOT/.jcy-agent-lock"

usage() {
    cat <<'EOF'
Usage:
  jcy_agent_guard.sh status
  jcy_agent_guard.sh acquire <agent> <task>
  jcy_agent_guard.sh release <agent>
EOF
}

show_lock() {
    if [[ -d "$LOCK_DIR" ]]; then
        echo "JCY2001D edit lock: BUSY"
        cat "$LOCK_DIR/metadata" 2>/dev/null || true
    else
        echo "JCY2001D edit lock: FREE"
    fi
}

command="${1:-status}"
case "$command" in
    status)
        show_lock
        ;;
    acquire)
        agent="${2:-}"
        task="${3:-}"
        if [[ -z "$agent" || -z "$task" ]]; then
            usage
            exit 64
        fi
        if ! mkdir "$LOCK_DIR" 2>/dev/null; then
            show_lock
            echo "Refusing concurrent edit. Ask the lock owner to finish or release it." >&2
            exit 2
        fi
        {
            printf 'owner=%s\n' "$agent"
            printf 'task=%s\n' "$task"
            printf 'host=%s\n' "$(hostname)"
            printf 'pid=%s\n' "$$"
            printf 'branch=%s\n' "$(git -C "$ROOT" branch --show-current)"
            printf 'started=%s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')"
        } > "$LOCK_DIR/metadata"
        echo "Lock acquired by $agent for: $task"
        ;;
    release)
        agent="${2:-}"
        if [[ -z "$agent" ]]; then
            usage
            exit 64
        fi
        if [[ ! -d "$LOCK_DIR" ]]; then
            echo "No lock is held."
            exit 0
        fi
        owner="$(sed -n 's/^owner=//p' "$LOCK_DIR/metadata" | head -n1)"
        if [[ "$owner" != "$agent" ]]; then
            echo "Refusing to release a lock held by '${owner:-unknown}'." >&2
            exit 3
        fi
        rm -f "$LOCK_DIR/metadata"
        rmdir "$LOCK_DIR"
        echo "Lock released by $agent"
        ;;
    *)
        usage
        exit 64
        ;;
esac
