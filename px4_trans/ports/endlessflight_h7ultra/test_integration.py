#!/usr/bin/env python3
"""Offline smoke test for vendor BSP / PX4 v1.16.0 staging (NOT a PX4 compile).

Run: python3 test_integration.py --vendor-zip /path/to/fc-h7-ultra-master.zip
"""
from __future__ import annotations
import argparse
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import integrate_px4_v1160 as port

def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--vendor-zip", required=True, type=Path)
    args = p.parse_args()
    archive = args.vendor_zip.expanduser().resolve()
    assert archive.is_file(), "missing manufacturer archive"
    with TemporaryDirectory(prefix="h7ultra-gslink-test-") as tmp:
        root = Path(tmp)
        px4 = root / "PX4-Autopilot"
        (px4 / ".git").mkdir(parents=True)
        (px4 / "src/modules").mkdir(parents=True)
        (px4 / "src/modules/Kconfig").write_text('rsource "*/Kconfig"\n')
        (px4 / "msg").mkdir()
        (px4 / "msg/CMakeLists.txt").write_text("set(msg_files\n\tHealthReport.msg\n)\n")
        source = Path(__file__).resolve().parent
        with patch.object(port.subprocess, "check_output", return_value=port.PX4_V1160+"\n"):
            staged, commit = port.plan(archive, px4, source)
            assert commit == port.PX4_V1160
            assert len(staged) == 33, f"unexpected staged file count: {len(staged)}"
            assert b"CONFIG_MODULES_GS_LINK=y" in staged[port.BOARD / "gslink.px4board"]
            assert b"GsLinkControl.msg" in staged[Path("msg/CMakeLists.txt")]
            assert b"GsLinkStatus.msg" in staged[Path("msg/CMakeLists.txt")]
            assert b"\nCONFIG_DRIVERS_MAGNETOMETER_QMC5883P=y" not in staged[port.BOARD / "default.px4board"]
            assert b"\nCONFIG_COMMON_INS=y" not in staged[port.BOARD / "default.px4board"]
            assert b"\nqmc5883p -I" not in staged[port.BOARD / "init/rc.board_sensors"]
            assert b"param set-default SYS_HAS_MAG 0" in staged[port.BOARD / "init/rc.board_defaults"]
            for path, payload in staged.items():
                target = px4 / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(payload)
            again, _ = port.plan(archive, px4, source)
            assert again == staged, "integration is not idempotent"
            (px4 / "src/modules/gs_link/GsLink.cpp").write_bytes(b"local changes")
            try:
                port.plan(archive, px4, source)
            except ValueError as exc:
                assert "refusing to overwrite" in str(exc)
            else:
                raise AssertionError("collision was not rejected")
            try:
                with patch.object(port.subprocess, "check_output", return_value="wrong\n"):
                    port.plan(archive, px4, source)
            except ValueError as exc:
                assert "official PX4" in str(exc)
            else:
                raise AssertionError("wrong PX4 commit was not rejected")
    print("PASS: pinned vendor BSP (24 files), Ryan GS-Link (7 files), uORB registration and bench target")
    print("PASS: idempotent, no overwrite of modified files, rejects incorrect PX4 version")
    print("NOTE: this tests staging only; no PX4 CMake/ARM compile, hardware, or flight tests")

if __name__ == "__main__":
    main()
