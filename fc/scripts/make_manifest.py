#!/usr/bin/env python3
"""Derive manifest.json from built firmware artifacts.

The manifest is read out of the artifacts rather than asserted alongside them:
the ArduPilot .apj file is JSON carrying its own board_id and git_identity, so
the board a binary may be flashed to and the commit it was built from are facts
the file already knows. The hashes are what the bench compares before flashing,
which is the mechanism that makes "which exact bits fly" a lookup rather than a
memory.

    fc/scripts/make_manifest.py build/CubeOrangePlus/bin [--out manifest.json]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git(*args: str) -> str:
    root = Path(__file__).resolve().parent.parent.parent
    p = subprocess.run(["git", "-C", str(root), *args],
                       capture_output=True, text=True)
    return p.stdout.strip()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bindir", help="build/<board>/bin")
    ap.add_argument("--out", default=None, help="default: <bindir>/manifest.json")
    ap.add_argument("--tag", default=None, help="release tag, if this is one")
    args = ap.parse_args(argv)

    bindir = Path(args.bindir)
    if not bindir.is_dir():
        print(f"no such directory: {bindir}", file=sys.stderr)
        return 2
    board = bindir.parent.name

    entry: dict[str, object] = {}
    apj = bindir / "arduplane.apj"
    if apj.exists():
        doc = json.loads(apj.read_text())
        entry["board_id"] = doc.get("board_id")
        entry["git_identity"] = doc.get("git_identity")
        entry["apj_sha256"] = sha256(apj)
    for name, key in (("arduplane.bin", "bin_sha256"),
                      ("arduplane.hex", "hex_sha256"),
                      ("arduplane.abin", "abin_sha256"),
                      ("arduplane", "elf_sha256")):
        f = bindir / name
        if f.exists() and f.is_file():
            entry[key] = sha256(f)

    manifest = {
        "tag": args.tag or git("describe", "--tags", "--exact-match") or None,
        "git_sha": git("rev-parse", "HEAD"),
        "base_tag": "Plane-4.7.0",
        "boards": {board: entry},
        "toolchain": "gcc-arm-none-eabi-10-2020-q4-major",
        "empy": "3.3.4",
    }
    out = Path(args.out) if args.out else bindir / "manifest.json"
    out.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"wrote {out}")
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
