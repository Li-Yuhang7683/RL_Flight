#!/usr/bin/env python3
"""Stage vendor EndlessFlight H7Ultra BSP and Ryan GS-Link in PX4 *v1.16.0*.

DRY RUN by default; use --apply only after reviewing. Produces a communications-
only BENCH variant; NEVER treat the result as flight-ready or flash untested.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
import zipfile

PX4_V1160 = '6ea3539157ca358c70a515878b77077af7d4611d'
VENDOR_BSP_SHA256 = 'b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27'
INNER_BSP = 'fc-h7-ultra-master/PX4/PX4_1.16.0/h743ultra.zip'
BOARD = Path('boards/endlessflight/h743ultra')
GS_SOURCE = {
    'gs_link/CMakeLists.txt': 'src/modules/gs_link/CMakeLists.txt',
    'gs_link/Kconfig': 'src/modules/gs_link/Kconfig',
    'gs_link/GsLink.cpp': 'src/modules/gs_link/GsLink.cpp',
    'gs_link/GsLink.hpp': 'src/modules/gs_link/GsLink.hpp',
    'gs_link/GsLinkProtocol.hpp': 'src/modules/gs_link/GsLinkProtocol.hpp',
    'msg/GsLinkControl.msg': 'msg/GsLinkControl.msg',
    'msg/GsLinkStatus.msg': 'msg/GsLinkStatus.msg',
}
VARIANT = '''# BENCH ONLY: EndlessFlight H7Ultra GS-Link v1 communications.
# Uses the ORIGINAL H7Ultra BSP with staged no-magnetometer adjustments.
# Keep stock USB QGC, manual RC, PX4 Commander and actuator safety checks.
# GS-Link does not auto-start; ttyS0 MAVLink remains until manually disabled.
CONFIG_MODULES_GS_LINK=y
# Optional modules removed to make space in limited STM32H743 flash.
# This variant is NOT a flight firmware.
CONFIG_COMMON_DIFFERENTIAL_PRESSURE=n
CONFIG_DRIVERS_UAVCAN=n
CONFIG_DRIVERS_TAP_ESC=n
CONFIG_MODULES_FW_ATT_CONTROL=n
CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL=n
CONFIG_MODULES_FW_POS_CONTROL=n
CONFIG_MODULES_FW_RATE_CONTROL=n
CONFIG_MODULES_VTOL_ATT_CONTROL=n
CONFIG_MODULES_GYRO_FFT=n
CONFIG_MODULES_PAYLOAD_DELIVERER=n
CONFIG_MODULES_LOCAL_POSITION_ESTIMATOR=n
CONFIG_MODULES_UXRCE_DDS_CLIENT=n
'''

def stop(msg: str) -> None:
    raise ValueError(msg)

def bsp_bytes(archive: Path) -> bytes:
    with zipfile.ZipFile(archive) as z:
        files = set(z.namelist())
        if INNER_BSP in files:
            data = z.read(INNER_BSP)
        elif 'h743ultra/default.px4board' in files:
            data = archive.read_bytes()
        else:
            stop('not the uploaded EndlessFlight master ZIP or h743ultra.zip')
    digest = hashlib.sha256(data).hexdigest()
    if digest != VENDOR_BSP_SHA256:
        stop('vendor BSP digest mismatch: ' + digest)
    return data

def vendor_files(archive: Path) -> dict[Path, bytes]:
    data = bsp_bytes(archive)
    entries: dict[Path, bytes] = {}
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for entry in z.infolist():
            if entry.is_dir():
                continue
            p = PurePosixPath(entry.filename)
            if len(p.parts) < 2 or p.parts[0] != 'h743ultra' or '..' in p.parts:
                stop('unsafe BSP entry ' + entry.filename)
            if entry.file_size > 1_000_000:
                stop('oversized BSP entry ' + entry.filename)
            dest = BOARD.joinpath(*p.parts[1:])
            if dest in entries:
                stop('duplicate BSP entry ' + entry.filename)
            entries[dest] = z.read(entry)
    required = ('default.px4board','firmware.prototype','src/board_config.h',
                'src/CMakeLists.txt','nuttx-config/nsh/defconfig',
                'init/rc.board_sensors','init/rc.board_defaults')
    if not all(BOARD / f in entries for f in required):
        stop('vendor BSP missing required board files')
    firmware = json.loads(entries[BOARD / 'firmware.prototype'])
    if firmware.get('board_id') != 1179 or 'EndlessFlight_H7ultra' not in firmware.get('description',''):
        stop('vendor H7Ultra board identity mismatch')
    if 'CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS0"' not in entries[BOARD/'default.px4board'].decode():
        stop('verified TELEM1 /dev/ttyS0 mapping changed')
    return entries

def bench_adapt(entries: dict[Path,bytes]) -> None:
    """Adapt ONLY staged copies. Vendor archive stays byte-exact."""
    cfg = BOARD / 'default.px4board'
    data = entries[cfg].decode()
    for line,why in (
        ('CONFIG_DRIVERS_MAGNETOMETER_QMC5883P=y','unsupported vendor-only QMC5883P driver'),
        ('CONFIG_COMMON_INS=y','unsupported vendor-only COMMON_INS flag'),
    ):
        if data.count(line) != 1:
            stop('expected one vendor Kconfig line: ' + line)
        data = data.replace(line, '# BENCH ONLY: '+why+'\n# '+line,1)
    entries[cfg] = data.encode()
    sensors = BOARD / 'init/rc.board_sensors'
    data = entries[sensors].decode()
    cmd = 'qmc5883p -I -a 0x2C -R 6 start'
    if data.count(cmd) != 1:
        stop('unexpected QMC5883P startup command')
    entries[sensors] = data.replace(cmd,'# BENCH ONLY: QMC5883P unavailable\n# '+cmd,1).encode()
    defaults = BOARD / 'init/rc.board_defaults'
    data = entries[defaults].decode()
    original = 'param set-default SYS_HAS_MAG 1'
    if data.count(original) != 1:
        stop('unexpected vendor SYS_HAS_MAG default')
    entries[defaults] = data.replace(original,
        '# BENCH ONLY: no QMC5883P driver\nparam set-default SYS_HAS_MAG 0',1).encode()

def plan(archive:Path, px4:Path, source:Path) -> tuple[dict[Path,bytes],str]:
    if not (px4/'.git').exists() or not (px4/'msg/CMakeLists.txt').is_file() or not (px4/'src/modules/Kconfig').is_file():
        stop('PX4 directory must be a complete Git source checkout')
    head = subprocess.check_output(['git','-C',str(px4),'rev-parse','HEAD'],text=True).strip()
    if head != PX4_V1160:
        stop('expected official PX4 v1.16.0 commit '+PX4_V1160+'; got '+head)
    if '*/Kconfig' not in (px4/'src/modules/Kconfig').read_text():
        stop('PX4 module Kconfig discovery layout changed')
    files = vendor_files(archive)
    bench_adapt(files)
    files[BOARD/'gslink.px4board'] = VARIANT.encode()
    for src,dest in GS_SOURCE.items():
        path = source/src
        if not path.is_file():
            stop('missing pinned Ryan GS-Link source '+str(path))
        files[Path(dest)] = path.read_bytes()
    messages = px4/'msg/CMakeLists.txt'
    text = messages.read_text()
    if 'set(msg_files' not in text or '\tHealthReport.msg' not in text:
        stop('unrecognized PX4 msg/CMakeLists.txt layout')
    for msg in ('GsLinkControl.msg','GsLinkStatus.msg'):
        if '\t'+msg not in text:
            text = text.replace('\tHealthReport.msg','\t'+msg+'\n\tHealthReport.msg',1)
    files[Path('msg/CMakeLists.txt')] = text.encode()
    conflicts = []
    for relative, bytes_ in files.items():
        dst = px4 / relative
        if dst.exists() and dst.read_bytes() != bytes_:
            # msg/CMakeLists.txt is deliberately updated in-place after its
            # exact known anchor has been verified; everything else immutable.
            if relative != Path('msg/CMakeLists.txt'):
                conflicts.append(str(relative))
    if conflicts:
        stop('refusing to overwrite existing different files: '+', '.join(conflicts))
    return files,head

def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-zip', type=Path, required=True)
    parser.add_argument('--px4-dir', type=Path, required=True)
    parser.add_argument('--apply', action='store_true', help='Write local PX4 source (default: dry run)')
    args=parser.parse_args()
    source=Path(__file__).resolve().parent
    try:
        files, head=plan(args.vendor_zip.expanduser().resolve(),args.px4_dir.expanduser().resolve(),source)
    except (OSError,ValueError,subprocess.CalledProcessError,zipfile.BadZipFile) as exc:
        parser.error(str(exc))
    px4=args.px4_dir.expanduser().resolve()
    print('OFFICIAL PX4 VERSION:',head)
    print('VENDOR BSP SHA256:',VENDOR_BSP_SHA256)
    print('TARGET BOARD: EndlessFlight_H7ultra; BOARD_ID:1179 (not unique)')
    print('BENCH-ONLY: QMC5883P and COMMON_INS removed from staged files; SYS_HAS_MAG=0')
    changes={}
    for k,v in files.items():
        p=px4/k
        if not p.exists() or p.read_bytes()!=v:
            changes[k]=v
            print(('UPDATE ' if p.exists() else 'ADD    ')+str(k))
    print(f'FILES: {len(files)}, CHANGES: {len(changes)}')
    if not args.apply:
        print('DRY RUN: nothing written. Re-run with --apply after review.')
        return 0
    for k,v in changes.items():
        p=px4/k
        p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes(v)
    print('STAGED. Not compiled, not flashed, not flight-ready.')
    print('Next in PX4 checkout: make endlessflight_h743ultra_gslink')
    return 0

if __name__=='__main__':
    sys.exit(main())
