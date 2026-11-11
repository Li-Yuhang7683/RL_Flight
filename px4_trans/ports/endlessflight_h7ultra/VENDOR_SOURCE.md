# Verified EndlessFlight H7Ultra source and integration status

Vendor Gitee repository:
https://gitee.com/endlessflight/fc-h7-ultra

The user uploaded `fc-h7-ultra-master.zip` on 2026-11-11.
Live Gitee browsing was unavailable; the ZIP contents were **directly inspected**.

## Distribution inventory

- Outer ZIP SHA-256: `9175a8738e712f23a860ada18cded7da120eaaf84ca3f5ca548aa5b3ee8ce7d0`
- Inner board archive: `PX4/PX4_1.16.0/h743ultra.zip`
- Inner SHA-256: `b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27`
- 24 board files including `src/board_config.h`, `src/hw_config.h`,
  `src/spi.cpp`, `src/i2c.cpp`, `src/timer_config.cpp`, NuttX
  `nsh/defconfig`, `firmware.prototype`, `init/rc.board_sensors`,
  `default.px4board` and a packaged vendor bootloader.
- Factory `endlessflight_h743ultra_default.px4` metadata:
  `board_id=1179`, `description=Firmware for the EndlessFlight_H7ultra board`,
  `git_identity=v1.16.0`,
  `git_hash=6ea3539157ca358c70a515878b77077af7d4611d`.
  This matches the `ver all` output on real hardware.
- **Important**: Ryan's MicoAir firmware.prototype also declares
  `board_id=1179`. Board ID alone cannot detect misflashing.

## Target hardware and current communication

EndlessFlight H7Ultra with STM32H743VIT6, IIM42652 on SPI2/SPI3,
SPA06 on I2C2, QMC5883P on I2C2; UART1 = TELEM1 =
`/dev/ttyS0`, RX1 Pin3, TX1 Pin4, GND Pin2, 5V Pin1.
Current PX4 v1.16.0 on hardware uses UART1 MAVLink at 921600;
USB `/dev/ttyACM0` is independent and kept for QGC.
For two USB-powered boards, leave H7Ultra UART pin1 5V disconnected.

## Compatibility versus pinned Ryan PX4 source

Ryan git submodule HEAD:
`3fef8242687cf733b590ff21c5b36779943299d1`,
based on his PX4 v1.16.2 modification.
The vendor board files target PX4 v1.16.0; API/ABI compatibility
still requires compilation, checks and hardware validation.

Ryan tree contains:
- `src/modules/gs_link/{CMakeLists.txt,Kconfig,GsLink.cpp,GsLink.hpp,GsLinkProtocol.hpp}`
- `msg/{GsLinkControl.msg,GsLinkStatus.msg}`
- IIM42652, SPA06 and SCH16T driver Kconfigs.

Pinned Ryan tree does **not** contain a QMC5883P driver. The
`gslink.px4board` bench variant explicitly disables its Kconfig,
while the vendor `rc.board_sensors` still attempts `qmc5883p start`.
This discrepancy must be resolved for a flight-worthy target.
Do not assume the presence of a factory factory binary supplies missing
source files.

## How to stage

From a **clean WSL2 checkout** of the user fork's
`feature/endlessflight-h7ultra-gs-link` branch:

```bash
python3 px4_trans/ports/endlessflight_h7ultra/apply_h7ultra_board.py \
 --vendor-zip /path/to/fc-h7-ultra-master.zip \
 --px4-dir px4_trans/px4
```

Review dry-run output, then add `--apply` to stage. The helper checks
the vendor archive SHA-256, board identity, UART pin mapping and
exact Ryan PX4 submodule commit before any writes. It refuses overwrites.
All modifications to `px4_trans/px4` are **local worktree changes**,
not automatically new Git commits in Ryan's separate PX4 repo.
No firmware, flight permission or QGC upload is produced by this step.

```bash
cd px4_trans/px4
make endlessflight_h743ultra_gslink
```

Build was **NOT executed** in this environment (no full PX4 toolchain/
submodule available locally). Preserve factory firmware and parameter
backup. Do NOT upload an unverified image to the flight controller.

## Deliberate functional boundary

Only GS-Link communication is integrated in this stage. It depends
on `gs_link_status` that is normally published by Ryan's `nn_control`.
This GS-Link-only bench variant does not port the safety gate, controller,
or ground-station actuator authority. It must never be used for flight.
