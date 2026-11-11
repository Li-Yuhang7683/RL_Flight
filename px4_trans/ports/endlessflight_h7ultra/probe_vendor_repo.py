#!/usr/bin/env python3
"""Read-only survey of a local EndlessFlight vendor repo or extracted package."""
from __future__ import annotations
from pathlib import Path
import argparse
import subprocess

def git(root: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(root), *args],
                            capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "(unavailable)"

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="Local clone or extracted vendor source")
    root = parser.parse_args().path.expanduser().resolve()
    if not root.is_dir():
        parser.error("Directory does not exist: " + str(root))
    print("VENDOR_ROOT:", root)
    print("GIT_REMOTE:", git(root, "remote", "-v"))
    print("GIT_HEAD:", git(root, "rev-parse", "HEAD"))
    print("GIT_BRANCH:", git(root, "branch", "--show-current"))
    print("TOP_LEVEL:", ", ".join(sorted(p.name for p in root.iterdir())[:80]))
    config_paths = sorted(root.glob("**/boards/*/*/default.px4board"))
    print("BOARD_CONFIGS:", len(config_paths))
    for p in config_paths[:60]:
        print("  ", p.relative_to(root))
    px4_roots = sorted({p.parents[3] for p in config_paths})
    for base in px4_roots:
        valid = (base / "src/modules").is_dir() and (base / "msg/CMakeLists.txt").is_file()
        print("PX4_TREE:", base, "COMPLETE_LAYOUT:", valid)
    if not config_paths:
        print("NO PX4 board-source layouts found; do not treat docs/firmware as BSP.")
    for pat in ("**/*.px4", "**/*.zip", "**/*.tar.gz", "**/.gitmodules"):
        candidates = sorted(root.glob(pat))
        print("CANDIDATES", pat, len(candidates))
        for p in candidates[:12]:
            print("  ", p.relative_to(root))
    if px4_roots and not any("endless" in str(p).lower() or "ultra" in str(p).lower()
                              for p in config_paths):
        print("WARNING: Found PX4 boards but not verified EndlessFlight H7Ultra target")
    print("NOTICE: File presence does not prove the source matches the running firmware.")

if __name__ == "__main__":
    main()
