#!/usr/bin/env python3
"""Regenerate identified-LQI golden vectors from the embedded model header.

The source identification JSON is not tracked in this repository.  This tool
keeps the host checks reproducible from the authoritative generated payload
that is actually compiled into the firmware.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "px4_trans/px4/src/modules/nn_control/LqiIdentifiedModel.hpp"
TOOLS = ROOT / "px4_trans/tools"


def parse_array(text: str, name: str) -> np.ndarray:
    match = re.search(
        rf"static constexpr float {re.escape(name)}\[\d+\]\s*=\s*\{{(.*?)\}};",
        text,
        re.DOTALL,
    )
    if match is None:
        raise ValueError(f"missing {name} in {HEADER}")
    values = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:e[-+]?\d+)?", match.group(1))
    return np.asarray(values, dtype=np.float32)


def load_model() -> dict[str, np.ndarray | np.float32]:
    text = HEADER.read_text(encoding="utf-8")
    model_block = text.split("static constexpr LqiCompositeControllerModel kModel {", 1)[1]
    scalars = re.findall(
        r"^\t([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:e[-+]?\d+)?)f,$",
        model_block,
        re.MULTILINE,
    )
    if len(scalars) != 9:
        raise ValueError(f"expected 9 scalar model values, got {len(scalars)}")
    (
        dt,
        upper_trim,
        lower_trim,
        upper_trim_rad_s,
        lower_trim_rad_s,
        rpm_to_rad_s,
        max_roll_angle,
        max_pitch_angle,
        max_yaw_rate,
    ) = (np.float32(value) for value in scalars)
    model = {
        "gain": parse_array(text, "kGain").reshape(4, 19),
        "motor_decay": parse_array(text, "kMotorDecay"),
        "motor_slope_eff": parse_array(text, "kMotorSlopeEff"),
        "latent_decay": parse_array(text, "kLatentDecay"),
        "latent_input": parse_array(text, "kLatentInput").reshape(3, 3, 3),
        "integral_limit": parse_array(text, "kIntegralLimit"),
        "motor_table": parse_array(text, "kMotorTable").reshape(7, 2),
        "manual_command_decay": parse_array(text, "kManualCommandDecay"),
        "dt": dt,
        "upper_trim": upper_trim,
        "lower_trim": lower_trim,
        "upper_trim_rad_s": upper_trim_rad_s,
        "lower_trim_rad_s": lower_trim_rad_s,
        "rpm_to_rad_s": rpm_to_rad_s,
        "max_roll_angle": max_roll_angle,
        "max_pitch_angle": max_pitch_angle,
        "max_yaw_rate": max_yaw_rate,
    }
    expected_shapes = {
        "gain": (4, 19),
        "motor_decay": (2,),
        "motor_slope_eff": (2,),
        "latent_decay": (3,),
        "latent_input": (3, 3, 3),
        "integral_limit": (3,),
        "motor_table": (7, 2),
        "manual_command_decay": (3,),
    }
    for name, shape in expected_shapes.items():
        if model[name].shape != shape:
            raise ValueError(f"{name}: expected {shape}, got {model[name].shape}")
    return model


def main() -> None:
    sys.path.insert(0, str(TOOLS))
    import generate_lqi_backend as generator

    model = load_model()
    vectors = generator.composite_golden_vectors(model)
    generator.write_composite_golden(vectors)
    print(
        f"regenerated {len(vectors)} identified-LQI vectors for checksum "
        f"0x{generator.composite_checksum(model):08x}"
    )


if __name__ == "__main__":
    main()
