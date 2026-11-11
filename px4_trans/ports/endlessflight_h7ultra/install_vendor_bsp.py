#!/usr/bin/env python3
"""Install a pinned EndlessFlight H7Ultra PX4 v1.16.0 BSP into a PX4 checkout.

Reads the *actual* vendor fc-h7-ultra-master.zip or its nested h743ultra.zip.
Preserves the original files except for the optional, explicitly COMMUNICATIONS-
ONLY gslink build variant. Never modifies PX4 serial/MAVLink parameters,
bootloader configuration, the Git submodule ref, or physical hardware.
"""
from __future__ import annotations
import argparse
import hashlib
import io
from pathlib import Path, PurePosixPath
import sys
import zipfile

PINNED_VENDOR_BSP_SHA256 = "b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27"
INNER_PATH = "fc-h7-ultra-master/PX4/PX4_1.16.0/h743ultra.zip"
BOARD_REL = Path("boards/endlessflight/h743ultra")
TARGET_BOARD_ID = 1179
VARIANT = (
    "# Communication-only GS-Link v1 target, NOT FLIGHT READY.\n"
    "# Vendor H7Ultra default.px4board is retained as base.\n"
    "# This overlay keeps stock PX4 flight controllers & USB MAVLink.\n"
    "CONFIG_MODULES_GS_LINK=y\n"
)

def fail(message: str) -> None:
    raise SystemExit("ERROR: " + message)

def select_bsp(raw: bytes) -> bytes:
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        names = z.namelist()
        if INNER_PATH in names:
            return z.read(INNER_PATH)
        if "h743ultra/default.px4board" in names:
            return raw
    fail("Expected Gitee master ZIP or nested h743ultra.zip")

def read_board_files(data: bytes) -> dict[Path, bytes]:
    entries = {}
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for item in z.infolist():
            if item.is_dir():
                continue
            name = PurePosixPath(item.filename)
            if not name.parts or name.parts[0] != "h743ultra" or ".." in name.parts:
                fail("Unsafe/non-BSP archive entry: " + item.filename)
            dest = Path(*name.parts[1:])
            if str(dest) in ("", ".") or dest in entries:
                fail("Duplicate/invalid BSP entry: " + item.filename)
            if item.file_size > 2_000_000:
                fail("Oversized unexpected BSP entry: " + item.filename)
            entries[dest] = z.read(item)
    for required in (
        Path("default.px4board"), Path("firmware.prototype"),
        Path("src/board_config.h"), Path("src/CMakeLists.txt"),
        Path("nuttx-config/nsh/defconfig"), Path("init/rc.board_defaults"),
    ):
        if required not in entries:
            fail("Incomplete vendor board package, missing " + str(required))
    if b'"board_id": 1179' not in entries[Path("firmware.prototype")]:
        fail("Unexpected vendor PX4 board ID (expected 1179)")
    if b'boards/endlessflight/h743ultra' not in entries[Path("nuttx-config/nsh/defconfig")]:
        fail("Unexpected BSP vendor/board path in NuttX config")
    return entries

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vendor-zip", type=Path, required=True)
    ap.add_argument("--px4-dir", type=Path, required=True)
    ap.add_argument("--apply", action="store_true", help="Write files; default is a dry run")
    args = ap.parse_args()

    if not args.vendor_zip.is_file():
        ap.error("Vendor ZIP not found")
    data = select_bsp(args.vendor_zip.read_bytes())
    digest = hashlib.sha256(data).hexdigest()
    if digest != PINNED_VENDOR_BSP_SHA256:
        fail("Vendor BSP SHA256 mismatch: " + digest
             + " (review new vendor revision before updating pin)")
    files = read_board_files(data)

    root = args.px4_dir.expanduser().resolve()
    if not (root / "msg/CMakeLists.txt").is_file() or not (root / "src/modules/Kconfig").is_file():
        fail("--px4-dir is not a complete PX4 source tree")
    if not (root / ".git").exists():
        fail("Use a Git checkout, not a firmware-only release")
    target = root / BOARD_REL
    if target.exists():
        fail("Board directory already exists; refusing to replace or partially merge it")

    # Ryan's pinned PX4 tree has the vendor IIM42652/SPA06/SCH16T drivers,
    # but no QMC5883P driver. Do not substitute QMC5883L (different sensor).
    mag_driver = root / "src/drivers/magnetometer/qmc5883p/Kconfig"
    has_qmc = mag_driver.is_file()
    variant = VARIANT
    if not has_qmc:
        # Bench-only: remove unsupported magnetometer startup and board flag.
        config = files[Path("default.px4board")].decode("utf-8")
        original = "CONFIG_DRIVERS_MAGNETOMETER_QMC5883P=y"
        if original not in config:
            fail("Unexpected vendor QMC5883P driver setting")
        files[Path("default.px4board")] = config.replace(original,
            "# QMC5883P driver missing from this PX4 tree: BENCH ONLY\n"
            "CONFIG_DRIVERS_MAGNETOMETER_QMC5883P=n").encode()
        sensors = files[Path("init/rc.board_sensors")].decode("utf-8")
        command = "qmc5883p -I -a 0x2C -R 6 start"
        if command not in sensors:
            fail("Unexpected vendor QMC5883P startup command")
        files[Path("init/rc.board_sensors")] = sensors.replace(
            command, "# " + command + " # no driver in chosen PX4 source; bench only").encode()
        variant += "# QMC5883P driver absent. No substitute mag; NEVER FLY THIS BUILD.\n"
    files[Path("gslink.px4board")] = variant.encode()

    print("Vendor BSP SHA256:", digest)
    print("Board ID:", TARGET_BOARD_ID)
    print("Target:", target)
    print("Files:", len(files), "including separate gslink.px4board")
    print("QMC5883P driver exists:", has_qmc)
    if not has_qmc:
        print("WARNING: MAG DRIVER MISSING; staging a BENCH-ONLY board overlay.")
        print("         Original vendor ZIP unchanged; temporary QMC adjustments visible in diff.")
    print("Will NOT change bootloader, submodule ref, or existing PX4 parameters.")
    if not args.apply:
        print("DRY RUN: no files written. Review before --apply.")
        return 0

    for rel, content in files.items():
        destination = target / rel
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
    print("Installed vendor BSP. Next run apply_gs_link.py (separately).")
    print("DO NOT FLASH: build and verify board target/ID + USB/RC first.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
