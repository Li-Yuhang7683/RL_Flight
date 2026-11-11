# EndlessFlight H7Ultra / GS-Link v1 — phase 1 staging

**Status: source staging only; NOT compiled, NOT a verified H7Ultra firmware, NOT flight-ready.**

This folder is the **H7Ultra integration staging area in Li-Yuhang7683/RL_Flight**.
The parent repository continues to pin its \`px4_trans/px4\` submodule to Ryan's
PX4 source; the submodule is NOT an EndlessFlight board tree. Do not change the
submodule pointer or flash a MicoAir binary onto H7Ultra.

## Facts established from hardware

- Target board reports \`HW arch: ENDLESSFLIGHT_H743ULTRA\`, PX4 \`v1.16.0\`,
  Git hash \`6ea3539157ca358c70a515878b77077af7d4611d\`.
- Vendor README: PX4 \`/dev/ttyS0 = TELEM1 = UART1\`. Connector pins:
  pin 1 5V (leave disconnected when USB-powered); pin 2 GND;
  pin 3 RX1 3.3V; pin 4 TX1 3.3V.
- Current hardware: MAVLink owns \`/dev/ttyS0 @921600\`, USB QGC uses
  \`/dev/ttyACM0\`. ESP32-C5 uses UART1 TX=GPIO11, RX=GPIO12, baud=921600,
  \`CONFIG_DB_GSLINK_FRAMING=y\` and UDP port 14550.
- Ryan's original GS-Link uses 4,000,000 baud and Ryan's MicoAir target.
  For this first H7Ultra bench step use 921600 on BOTH sides; 4M must be
  tested separately and is not a prerequisite for protocol compatibility.

## Pinned, actual implementation sources

Copied verbatim from
[Ryan-5853/PX4-TeleaiTrans at 3fef8242687cf733b590ff21c5b36779943299d1](https://github.com/Ryan-5853/PX4-TeleaiTrans/tree/3fef8242687cf733b590ff21c5b36779943299d1):

- \`gs_link/CMakeLists.txt\`, \`gs_link/Kconfig\`
- \`gs_link/GsLink.cpp\`, \`gs_link/GsLink.hpp\`, \`gs_link/GsLinkProtocol.hpp\`
- \`msg/GsLinkControl.msg\`, \`msg/GsLinkStatus.msg\`

The protocol is **GS-Link v1**, not MAVLink. Transport frames use
\`A5 5A\` magic, version 1, 10-byte header and CRC16. Ryan's
[Dock_ground](https://github.com/Ryan-5853/Dock_ground) receives framed
UDP datagrams at PC port 39050 after sending its GCS heartbeat to C5:14550.

## Source dependency / limitation

The uploaded EndlessFlight README is **hardware documentation, not PX4 BSP
source**. The exact vendor \`boards/.../default.px4board\`, board configs,
boot scripts, NuttX defconfig, board ID and driver definitions for firmware
\`6ea3539...\` have NOT yet been recovered or independently verified.

**Never copy Ryan's MicoAir \`boards/micoair/h743-v2\` into H7Ultra.**
The current H7Ultra PX4 1.16.0 binary is NOT evidence that a compatible
board source tree exists in Ryan's submodule. Ryan's GS-Link module was
read from his v1.16.2-based tree, so source API compatibility must also be
checked against the exact EndlessFlight vendor source.

## Apply to a separate, genuine H7Ultra PX4 source checkout

Do NOT run against \`px4_trans/px4\` unless that tree independently contains
the exact EndlessFlight H7Ultra BSP. Given a manufacturer PX4 git checkout:

\`\`\`bash
# Run from your fork's RL_Flight root. Discover actual board path first:
find /path/to/endlessflight-px4/boards -name default.px4board | grep -i -E 'endless|h7ultra'

python3 px4_trans/ports/endlessflight_h7ultra/apply_gs_link.py \
  --px4-dir /path/to/endlessflight-px4 \
  --board-dir boards/<actual-vendor>/<actual-board>

# Check printed diff/plan before deliberately writing:
python3 px4_trans/ports/endlessflight_h7ultra/apply_gs_link.py \
  --px4-dir /path/to/endlessflight-px4 \
  --board-dir boards/<actual-vendor>/<actual-board> --apply
\`\`\`

The script refuses missing/incorrect BSPs, detects conflicting files,
installs Ryan's exact module and uORB messages, updates \`msg/CMakeLists.txt\`
and adds a **separate \`gslink.px4board\` variant** enabling only
\`CONFIG_MODULES_GS_LINK=y\`. It does not remove stock flight control,
rewrite board defaults, configure UART ownership, enable RC override,
modify PX4 boot scripts, arm aircraft, or flash hardware.

Before building, resolve 1.16.0 versus Ryan 1.16.2 compile differences.
The actual board target naming comes from the vendor tree; do not guess it.
Review scripts and board startup. Compile offline, then inspect generated
\`.px4\` metadata for matching H7Ultra board ID before any bench flash.

## Runtime bench plan (ONLY after a verified H7Ultra build)

1. Remove props / disconnect motor power; back up current vendor firmware
   and PX4 parameters. Keep the USB QGC connection.
2. On the verified test firmware, stop TELEM1 MAVLink ownership by setting
   \`MAV_0_CONFIG=0\`, save and reboot. Check \`mavlink status\`: USB must
   remain available and no \`/dev/ttyS0\` MAVLink instance may remain.
3. Start **only the communications module**, manually in NSH:
   \`gs_link start -d /dev/ttyS0 -b 921600\`
   and check \`gs_link status\`. Do NOT auto-start the module yet.
4. Cross UART wiring (H7 TX1 -> ESP GPIO12 RX, H7 RX1 <- ESP GPIO11 TX,
   common GND; leave H7 UART1 5V disconnected).
5. Check GS-Link packet counters/CRC, ESP frame stats and Dock_ground
   telemetry on PC. The original Ryan \`gs_link\` subscribes to
   \`gs_link_status\` normally published by \`nn_control\`; this phase
   DOES NOT port that controller or reproduce its RC safety gate.
   Therefore **no ground-station control or flying is authorized**.

## Subsequent phase

After phase-1 communications compile and real UART bench tests succeed,
port or independently implement the \`GroundStationGate\`/\`nn_control\`
publisher and verify all RC, arming, actuator, failsafe and HIL boundaries.
No GS-Link packet alone grants control authority.
