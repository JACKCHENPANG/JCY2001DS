#!/usr/bin/env python3
"""Single-owner firmware version helper for the JCY2001D release worktree."""

from __future__ import annotations

import argparse
import datetime as dt
import re
import subprocess
import sys
from pathlib import Path

VERSION_RE = re.compile(
    r'(#define JCY1000_FIRMWARE_VERSION\s+")V(?P<core>\d+\.\d+\.\d+)-A(?P<revision>\d+)(")'
)
TAG_RE = re.compile(
    r'(#define JCY1000_BUILD_TAG\s*\\\s*\n\s*")[^"]*(")'
)


def git_root() -> Path:
    return Path(
        subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], text=True
        ).strip()
    )


def canonical_root(root: Path) -> Path:
    lines = subprocess.check_output(
        ["git", "worktree", "list", "--porcelain"], cwd=root, text=True
    ).splitlines()
    for line in lines:
        if line.startswith("worktree "):
            return Path(line.removeprefix("worktree "))
    raise RuntimeError("cannot determine canonical worktree")


def source_path(root: Path) -> Path:
    path = root / "src" / "main.cpp"
    if not path.is_file():
        raise RuntimeError(f"firmware source not found: {path}")
    return path


def read_version(text: str) -> tuple[str, int]:
    match = VERSION_RE.search(text)
    if not match:
        raise RuntimeError("JCY1000_FIRMWARE_VERSION definition not found")
    return match.group("core"), int(match.group("revision"))


def require_owner_lock(root: Path, owner: str) -> None:
    metadata = root / ".jcy-agent-lock" / "metadata"
    if not metadata.is_file():
        raise RuntimeError("release lock missing; acquire it before bumping")
    recorded = next(
        (line.removeprefix("owner=") for line in metadata.read_text().splitlines()
         if line.startswith("owner=")),
        "",
    )
    if recorded != owner:
        raise RuntimeError(
            f"release lock belongs to '{recorded or 'unknown'}', not '{owner}'"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="print current firmware version")
    sub.add_parser("next", help="print the next A revision")
    bump = sub.add_parser("bump", help="increment A revision in canonical tree")
    bump.add_argument("--owner", required=True, help="holder of the edit lock")
    bump.add_argument("--note", required=True, help="short release note for build tag")
    args = parser.parse_args()

    root = git_root()
    source = source_path(root)
    text = source.read_text()
    core, revision = read_version(text)
    current = f"V{core}-A{revision}"
    next_version = f"V{core}-A{revision + 1}"

    if args.command == "status":
        print(current)
        return 0
    if args.command == "next":
        print(next_version)
        return 0

    canonical = canonical_root(root)
    if root != canonical:
        raise RuntimeError(
            f"version bump is only allowed in canonical worktree: {canonical}"
        )
    require_owner_lock(root, args.owner)
    note = args.note.strip()
    if not note:
        raise RuntimeError("release note must not be empty")
    updated, count = VERSION_RE.subn(
        lambda match: f'{match.group(1)}{next_version}{match.group(4)}',
        text,
        count=1,
    )
    if count != 1:
        raise RuntimeError("failed to update firmware version")
    build_tag = f"{dt.date.today().isoformat()} {next_version} {note}"
    updated, count = TAG_RE.subn(
        lambda match: f'{match.group(1)}{build_tag}{match.group(2)}',
        updated,
        count=1,
    )
    if count != 1:
        raise RuntimeError("failed to update firmware build tag")
    source.write_text(updated)
    print(f"{current} -> {next_version}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
