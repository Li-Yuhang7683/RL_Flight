#!/usr/bin/env python3
"""Stage original EndlessFlight H7Ultra board source into Ryan's PX4 checkout.

Use with an unmodified Ryan PX4 submodule at the pinned commit. Does NOT
change vendor bootloaders, flash hardware, or modify RL_Flight main branch.
Only writes after --apply. Refuses overwriting non-identical board files.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import zipfile

PINNED_RYAN = "3fef8242687cf733b590ff21c5b36779943299d1"
INNER_NAME = "fc-h7-ultra-master/PX4/PX4_1.16.0/h743ultra.zip"
INNER_SHA256 = "b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27"
VENDOR_BOARD = "boards/endlessflight/h743ultra"
BOARD_ID = 1179
GS_VARIANT = """# EndlessFlight H7Ultra GS-Link communication-only bench variant.
# Inherit a BENCH-ONLY adapted vendor default.px4board and stock controllers.
# Does not auto-start GS-Link or grant ground-station control authority.
CONFIG_MODULES_GS_LINK=y
"""
# Preserve the vendor ZIP byte-for-byte. These changes affect only the
# staged bench checkout, and must NEVER be mistaken for a flight BSP.
UNSUPPORTED = {
    "CONFIG_DRIVERS_MAGNETOMETER_QMC5883P=y": "QMC5883P driver absent in pinned Ryan PX4",
    "CONFIG_COMMON_INS=y": "COMMON_INS Kconfig symbol absent in pinned Ryan PX4",
}

def adapt_bench_board(files: dict[str, bytes]) -> dict[str, bytes]:
    config = files["default.px4board"].decode("utf-8")
    for key, reason in UNSUPPORTED.items():
        if config.count(key) != 1:
            raise ValueError("Expected exactly one vendor config entry: " + key)
        config = config.replace(key, "# BENCH ONLY: " + reason + "\n# " + key, 1)
    files["default.px4board"] = config.encode("utf-8")
    sensors = files["init/rc.board_sensors"].decode("utf-8")
    cmd = "qmc5883p -I -a 0x2C -R 6 start"
    if sensors.count(cmd) != 1:
        raise ValueError("Unexpected vendor QMC5883P startup script")
    files["init/rc.board_sensors"] = sensors.replace(
        cmd, "# BENCH ONLY: QMC5883P driver absent\n# " + cmd, 1
    ).encode("utf-8")
    defaults = files["init/rc.board_defaults"].decode("utf-8")
    old = "param set-default SYS_HAS_MAG 1"
    if defaults.count(old) != 1:
        raise ValueError("Unexpected SYS_HAS_MAG vendor default")
    files["init/rc.board_defaults"] = defaults.replace(
        old,
        "# BENCH ONLY: no magnetometer driver in pinned Ryan PX4\n"
        "param set-default SYS_HAS_MAG 0", 1
    ).encode("utf-8")
    return files

def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def board_from_zip(path: Path) -> dict[str, bytes]:
    with zipfile.ZipFile(path) as outer:
        names = set(outer.namelist())
        if INNER_NAME in names:
            payload = outer.read(INNER_NAME)
        elif "h743ultra/default.px4board" in names:
            payload = path.read_bytes()
        else:
            raise ValueError("Expected vendor fc-h7-ultra-master.zip or h743ultra.zip")
    if sha256(payload) != INNER_SHA256:
        raise ValueError("Unexpected vendor BSP zip SHA-256: " + sha256(payload))
    files = {}
    with zipfile.ZipFile(io.BytesIO(payload)) as board:
        for member in board.infolist():
            if member.is_dir():
                continue
            rel = Path(member.filename)
            if rel.is_absolute() or len(rel.parts) < 2 or rel.parts[0] != "h743ultra" \
                    or ".." in rel.parts:
                raise ValueError("Unsafe vendor board archive path: " + member.filename)
            raw = board.read(member)
            if len(raw) > 1_000_000:
                raise ValueError("Unexpected oversized vendor board file")
            files[str(Path(*rel.parts[1:]))] = raw
    required = ("default.px4board", "firmware.prototype", "src/board_config.h",
                "nuttx-config/nsh/defconfig", "init/rc.board_sensors",
                "init/rc.board_defaults")
    if any(k not in files for k in required):
        raise ValueError("Incomplete vendor board package")
    metadata = json.loads(files["firmware.prototype"].decode())
    if metadata.get("board_id") != BOARD_ID or "EndlessFlight" not in metadata.get("description", ""):
        raise ValueError("Unexpected vendor H7Ultra identity")
    if 'CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS0"' not in files["default.px4board"].decode():
        raise ValueError("Vendor UART1 mapping differs from verified hardware")
    return files

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor-zip", required=True, type=Path,
                        help="Downloaded manufacturer fc-h7-ultra-master.zip")
    parser.add_argument("--px4-dir", required=True, type=Path,
                        help="Ryan PX4 submodule checkout, px4_trans/px4")
    parser.add_argument("--apply", action="store_true", help="Write (default dry-run)")
    args = parser.parse_args()
    px4 = args.px4_dir.expanduser().resolve()
    z = args.vendor_zip.expanduser().resolve()
    if not z.is_file():
        parser.error("Vendor archive not found: " + str(z))
    if not (px4 / "src/modules/gs_link/GsLink.cpp").is_file():
        parser.error("Expected Ryan GS-Link module missing in PX4 source")
    try:
        head = subprocess.check_output(["git", "-C", str(px4), "rev-parse", "HEAD"],
                                       text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        parser.error("PX4 source must be a Git checkout")
    if head != PINNED_RYAN:
        parser.error("PX4 submodule version differs from pinned GS-Link source: " + head)
    try:
        files = board_from_zip(z)
    except (ValueError, OSError, zipfile.BadZipFile) as error:
        parser.error(str(error))
    files = adapt_bench_board(files)
    files["gslink.px4board"] = GS_VARIANT.encode()
    dst = px4 / VENDOR_BOARD
    changed = [k for k, data in files.items()
               if (dst / k).exists() and (dst / k).read_bytes() != data]
    if changed:
        parser.error("Refusing to overwrite modified BSP files: " + ", ".join(changed))
    print("Ryan PX4 commit:", head)\n    print("BENCH: QMC5883P/COMMON_INS unsupported; skip QMC startup; SYS_HAS_MAG=0")
    print("Vendor board:", VENDOR_BOARD, "ID:", BOARD_ID,
          "(shared ID with MicoAir; validate board name too)")
    for k in sorted(files):
        print(("EXISTS " if (dst / k).exists() else "ADD    ") + k)
    if not args.apply:
        print("DRY-RUN ONLY: no source files changed. Use --apply after review.")
        return 0
    for k, content in files.items():
        target = dst / k
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
    print("Staged BENCH-ONLY vendor BSP and gslink.px4board; no firmware built or flashed.")
    print("Next (after compatibility review): make endlessflight_h743ultra_gslink")\n    print("NOT flight-ready. No compiler, HIL, RC, UART or failsafe validation.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
