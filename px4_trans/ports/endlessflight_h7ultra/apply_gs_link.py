#!/usr/bin/env python3
"""Stage the pinned Ryan GS-Link module in an *existing* EndlessFlight PX4 tree.

Dry-run by default. Does not change the flight controller, ROMFS boot scripts,
saved PX4 parameters, or the RL_Flight PX4 git submodule pointer.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
COPY = {
    "gs_link/CMakeLists.txt": "src/modules/gs_link/CMakeLists.txt",
    "gs_link/Kconfig": "src/modules/gs_link/Kconfig",
    "gs_link/GsLink.cpp": "src/modules/gs_link/GsLink.cpp",
    "gs_link/GsLink.hpp": "src/modules/gs_link/GsLink.hpp",
    "gs_link/GsLinkProtocol.hpp": "src/modules/gs_link/GsLinkProtocol.hpp",
    "msg/GsLinkControl.msg": "msg/GsLinkControl.msg",
    "msg/GsLinkStatus.msg": "msg/GsLinkStatus.msg",
}
VARIANT = (
    "# EndlessFlight H7Ultra: communication-only GS-Link v1 build variant.\n"
    "# Inherits the vendor default.px4board; keeps USB MAVLink and stock controls.\n"
    "# No automatic port takeover or arming/control authority is added.\n"
    "CONFIG_MODULES_GS_LINK=y\n"
)

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--px4-dir", required=True, type=Path,
                    help="EXISTING EndlessFlight PX4 v1.16.x source directory")
    ap.add_argument("--board-dir", required=True,
                    help="Path relative to PX4 root, e.g. boards/<vendor>/<board>")
    ap.add_argument("--apply", action="store_true",
                    help="Write after validating all inputs (default: dry-run)")
    args = ap.parse_args()

    px4 = args.px4_dir.expanduser().resolve()
    board_rel = Path(args.board_dir)
    if board_rel.is_absolute() or ".." in board_rel.parts or not board_rel.parts \
            or board_rel.parts[0] != "boards":
        ap.error("--board-dir must be a relative path under boards/")
    if not (px4 / ".git").exists() or not (px4 / "msg/CMakeLists.txt").is_file():
        ap.error("--px4-dir must be a complete PX4 git checkout")
    board_dir = (px4 / board_rel).resolve()
    if not board_dir.is_relative_to(px4) or not (board_dir / "default.px4board").is_file():
        ap.error("EndlessFlight default.px4board not found at --board-dir")
    normalized = str(board_rel).lower().replace("-", "").replace("_", "")
    if "endless" not in normalized or not ("h7" in normalized or "743" in normalized):
        ap.error("Refusing non-EndlessFlight H7Ultra board path")
    kconfig = px4 / "src/modules/Kconfig"
    if not kconfig.is_file() or "*/Kconfig" not in kconfig.read_text():
        ap.error("Unknown PX4 module Kconfig layout; do not patch blindly")
    message_list = px4 / "msg/CMakeLists.txt"
    original = message_list.read_text()
    if "set(msg_files" not in original:
        ap.error("Unexpected msg/CMakeLists.txt layout")
    updated = original
    for name in ("GsLinkControl.msg", "GsLinkStatus.msg"):
        if name not in updated:
            anchor = "\tHealthReport.msg"
            if anchor not in updated:
                ap.error("Could not find uORB msg insertion anchor")
            updated = updated.replace(anchor, f"\t{name}\n" + anchor, 1)

    target_files = {px4 / dst: (HERE / src).read_bytes()
                    for src, dst in COPY.items()}
    target_files[board_dir / "gslink.px4board"] = VARIANT.encode()
    target_files[message_list] = updated.encode()
    conflicts = [str(path) for path, content in target_files.items()
                 if path.exists() and path.read_bytes() != content
                 and path != message_list]
    if conflicts:
        print("STOP: refusing to overwrite non-identical existing files:", file=sys.stderr)
        for path in conflicts:
            print("  " + path, file=sys.stderr)
        return 2

    print(f"PX4 source: {px4}")
    print(f"Board: {board_dir}")
    print("Source requirement: vendor EndlessFlight H7Ultra PX4 1.16.x tree, "
          "not verified by this script.")
    print("Changes:")
    for path, content in target_files.items():
        current = path.read_bytes() if path.exists() else None
        if current != content:
            print(f"  {'UPDATE' if current is not None else 'ADD'} {path.relative_to(px4)}")
    if not args.apply:
        print("DRY RUN: no files written. Add --apply only after reviewing vendor source.")
        return 0

    for path, content in target_files.items():
        if path.read_bytes() == content if path.exists() else False:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    print("Sources staged. No firmware built or flashed.")
    print("NEXT: inspect API compatibility with vendor PX4 1.16.0; then try")
    print(f"  make {board_rel.parts[1]}_{board_rel.parts[2]}_gslink")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
