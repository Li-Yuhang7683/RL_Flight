# EndlessFlight H7Ultra / GS-Link v1 — phase 1 staging

## 2026-11-11 update: genuine vendor BSP found in uploaded ZIP

The user supplied `fc-h7-ultra-master.zip`. Inside it:
`PX4/PX4_1.16.0/h743ultra.zip` (SHA256
`b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27`).
It contains **24 vendor board files** under `h743ultra/`, including
`default.px4board`, `firmware.prototype`, NuttX config, scripts,
SPI/I2C/timer hardware files and vendor bootloader.

The factory `.px4` firmware identifies as
`EndlessFlight_H7ultra`, `board_id=1179`, PX4 `v1.16.0`,
`git_hash=6ea3539157ca358c70a515878b77077af7d4611d`:
this **matches the hardware NSH**. Caution: Ryan's MicoAir firmware
prototype also uses board ID 1179; NEVER identify boards by board ID alone.

The pinned `px4_trans/px4` submodule points to Ryan's **PX4 v1.16.2-based**
repository, not the vendor's v1.16.0 core. The vendor ZIP supplies board
support files, **not the entire matching PX4 source tree**. This initial
integration merges vendor BSP files into Ryan's pinned PX4 checkout and
enables the existing GS-Link module through a separate
`gslink.px4board` variant; this is a version port and must be compiled
and validated before any flashing. Stock PX4 controllers remain enabled,
USB QGC is retained, and UART1 takeover is manual only.

**Read-only dry run / explicit apply (inside WSL2, from RL_Flight root):**

```bash
git fetch origin feature/endlessflight-h7ultra-gs-link
git switch feature/endlessflight-h7ultra-gs-link
git submodule update --init px4_trans/px4
git -C px4_trans/px4 rev-parse HEAD
# Expected: 3fef8242687cf733b590ff21c5b36779943299d1
python3 px4_trans/ports/endlessflight_h7ultra/apply_h7ultra_board.py \
  --vendor-zip /path/to/fc-h7-ultra-master.zip \
  --px4-dir px4_trans/px4
# Review plan; only if correct:
python3 px4_trans/ports/endlessflight_h7ultra/apply_h7ultra_board.py \
  --vendor-zip /path/to/fc-h7-ultra-master.zip \
  --px4-dir px4_trans/px4 --apply
```

NOTE: replace `/path/to/` with the real location. The script refuses
a non-matching Ryan submodule SHA or unknown vendor ZIP, does not modify
original default.px4board, and does NOT automatically start GS-Link.
Copying files into the PX4 submodule checkout causes a deliberately dirty
local submodule worktree; only the **source staging and script** are
committed to the parent RL_Flight branch.

After reviewing compile compatibility, try:
```bash
cd px4_trans/px4
make endlessflight_h743ultra_gslink
```
**Not yet compiled**. The GS-Link variant turns off the unsupported
`CONFIG_DRIVERS_MAGNETOMETER_QMC5883P`, but the original
`rc.board_sensors` still attempts to start `qmc5883p`. Ryan has
IIM42652, SPA06 and SCH16T drivers but not QMC5883P in his pinned tree.
This variant is **BENCH-ONLY / NOT FLIGHT-READY**. Do not arm.

See [VENDOR_SOURCE.md](VENDOR_SOURCE.md) for the source integrity and
compatibility details, and [SOURCE_MANIFEST.md](SOURCE_MANIFEST.md) for
the pinned Ryan GS-Link sources.

## Facts established from hardware

- Target board reports `HW arch: ENDLESSFLIGHT_H743ULTRA`, PX4 `v1.16.0`,
  Git hash `6ea3539157ca358c70a515878b77077af7d4611d`.
- Vendor README: PX4 `/dev/ttyS0 = TELEM1 = UART1`. Connector pins:
  pin 1 5V (leave disconnected when USB-powered); pin 2 GND;
  pin 3 RX1 3.3V; pin 4 TX1 3.3V.
- Current hardware: MAVLink owns `/dev/ttyS0 @921600`, USB QGC uses
  `/dev/ttyACM0`. ESP32-C5 uses UART1 TX=GPIO11, RX=GPIO12, baud=921600,
  `CONFIG_DB_GSLINK_FRAMING=y` and UDP port 14550.
- Ryan's original GS-Link uses 4,000,000 baud and Ryan's MicoAir target.
  For this first H7Ultra bench step use 921600 on BOTH sides; 4M must be
  tested separately and is not a prerequisite for protocol compatibility.

## Pinned, actual implementation sources

Copied verbatim from
[Ryan-5853/PX4-TeleaiTrans at 3fef8242687cf733b590ff21c5b36779943299d1](https://github.com/Ryan-5853/PX4-TeleaiTrans/tree/3fef8242687cf733b590ff21c5b36779943299d1):

- `gs_link/CMakeLists.txt`, `gs_link/Kconfig`
- `gs_link/GsLink.cpp`, `gs_link/GsLink.hpp`, `gs_link/GsLinkProtocol.hpp`
- `msg/GsLinkControl.msg`, `msg/GsLinkStatus.msg`

The protocol is **GS-Link v1**, not MAVLink. Transport frames use
`A5 5A` magic, version 1, 10-byte header and CRC16. Ryan's
[Dock_ground](https://github.com/Ryan-5853/Dock_ground) receives framed
UDP datagrams at PC port 39050 after sending its GCS heartbeat to C5:14550.

## Applying to the supplied vendor BSP

Use the instructions at the top of this file with the vendor ZIP;
the previously provided `apply_gs_link.py` is an alternative helper for a
**complete vendor PX4 git checkout**, and is *not* the recommended path
for this archive. The actual board path is
`boards/endlessflight/h743ultra` and the bench-only build variant is
`endlessflight_h743ultra_gslink`.

## Runtime bench plan (ONLY after a verified H7Ultra build)

1. Remove props / disconnect motor power; back up current vendor firmware
   and PX4 parameters. Keep the USB QGC connection.
2. On the verified test firmware, stop TELEM1 MAVLink ownership by setting
   `MAV_0_CONFIG=0`, save and reboot. Check `mavlink status`: USB must
   remain available and no `/dev/ttyS0` MAVLink instance may remain.
3. Start **only the communications module**, manually in NSH:
   `gs_link start -d /dev/ttyS0 -b 921600`
   and check `gs_link status`. Do NOT auto-start the module yet.
4. Cross UART wiring (H7 TX1 -> ESP GPIO12 RX, H7 RX1 <- ESP GPIO11 TX,
   common GND; leave H7 UART1 5V disconnected).
5. Check GS-Link packet counters/CRC, ESP frame stats and Dock_ground
   telemetry on PC. The original Ryan `gs_link` subscribes to
   `gs_link_status` normally published by `nn_control`; this phase
   DOES NOT port that controller or reproduce its RC safety gate.
   Therefore **no ground-station control or flying is authorized**.

## Subsequent phase

After phase-1 communications compile and real UART bench tests succeed,
port or independently implement the `GroundStationGate`/`nn_control`
publisher and verify all RC, arming, actuator, failsafe and HIL boundaries.
No GS-Link packet alone grants control authority.
