# EndlessFlight H7Ultra vendor source — access and provenance

Vendor repository provided by aircraft owner:  
https://gitee.com/endlessflight/fc-h7-ultra

## Verified from vendor README supplied by user (not from live Gitee)

- Aircraft board: EndlessFlight H7Ultra / STM32H743VIT6.
- Sensors: two IIM42652 IMUs (SPI2/SPI3), SPA06-003 barometer (I2C2),
  QMC5883P magnetometer (I2C2).
- PX4 serial mapping: `/dev/ttyS0` = TELEM1 = UART1;
  `/dev/ttyACM0` = USB; UART1 connector pin1=5V, pin2=GND,
  pin3=RX1 (3.3V), pin4=TX1 (3.3V).
- Running hardware identified by NSH as `ENDLESSFLIGHT_H743ULTRA`,
  PX4 v1.16.0, `6ea3539157ca358c70a515878b77077af7d4611d`.
- UART1 on current firmware is MAVLink `/dev/ttyS0 @921600`.
- This README is a hardware/pinout/FAQ document; it does NOT itself
  contain the board's PX4 NuttX configuration or compilation target.

## Live Gitee inspection status

Attempted direct web page, Gitee API, and README/raw URL access on
2026-11-11, but access failed in this environment. Therefore the contents
of the **remaining Gitee repository** have NOT been inspected; do not
conclude it lacks source code solely from its README.

## Next verification: clone in WSL2 and inspect locally

Launch **Windows Terminal / PowerShell**, then enter WSL:

```powershell
wsl -d Ubuntu-24.04
```

Then run **inside Ubuntu**:

```bash
cd ~/projects
git clone https://gitee.com/endlessflight/fc-h7-ultra.git
cd ~/projects/RL_Flight
git fetch origin
git switch feature/endlessflight-h7ultra-gs-link
python3 px4_trans/ports/endlessflight_h7ultra/probe_vendor_repo.py \
  ~/projects/fc-h7-ultra
```

The probe is read-only. It prints git commit, detected full PX4 tree,
available PX4 board configs, and candidate nested source archives/
repositories. Do not run `apply_gs_link.py --apply` until a complete,
correct target board source is verified. **Do not flash firmware.**

If the vendor Gitee project contains only hardware documentation and
prebuilt .px4 binaries, obtain the matching source tree from EndlessFlight,
including `boards/<vendor>/<target>/`, PX4 base commit, board identity,
NuttX and serial/DMA settings, and sensor drivers.

## Integration policy

Stage the real H7Ultra BSP as a **separate checkout/source** first;
Ryan's existing PX4 git submodule pinned to his MicoAir board is not
interchangeable with the manufacturer's target. Keep both version lines
identifiable; do not change Ryan's submodule pointer without verifying
an exact H7Ultra-compatible PX4 repository.

Initial scope: compile GS-Link communication-only module; keep regular
PX4 flight control and USB QGC. Porting nn_control, RC control authority,
and flight safety is a separate later phase.
