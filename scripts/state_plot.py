# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt

GT_COLOR = "#000000"
FG_COLOR = "#55A868"

LAYOUT = [
    (["x", "y", "z"], ["X (m)", "Y (m)", "Z (m)"]),
    (["roll", "pitch", "yaw"], ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"]),
    (["vx", "vy", "vz"], ["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"]),
    (
        ["accel_bias_x", "accel_bias_y", "accel_bias_z"],
        ["Accel Bias X", "Accel Bias Y", "Accel Bias Z"],
    ),
    (
        ["gyro_bias_x", "gyro_bias_y", "gyro_bias_z"],
        ["Gyro Bias X", "Gyro Bias Y", "Gyro Bias Z"],
    ),
    (
        ["mag_bias_x", "mag_bias_y", "mag_bias_z"],
        ["Mag Bias X (T)", "Mag Bias Y (T)", "Mag Bias Z (T)"],
    ),
]

NEIGHBOR_LAYOUT = [
    *LAYOUT[:2],
    (["delta_x", "delta_y", "delta_z"], ["Delta X (m)", "Delta Y (m)", "Delta Z (m)"]),
    (
        ["delta_roll", "delta_pitch", "delta_yaw"],
        ["Delta Roll (rad)", "Delta Pitch (rad)", "Delta Yaw (rad)"],
    ),
]


def _mask_gaps(
    t: npt.NDArray[np.float64], vals: npt.NDArray[np.float64]
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    if len(t) < 2:
        return t, vals
    dts = np.diff(t)
    gaps = np.where(dts > max(float(np.median(dts)) * 5.0, 0.5))[0] + 1
    if not len(gaps):
        return t, vals
    return np.insert(t, gaps, np.nan), np.insert(vals, gaps, np.nan)


def plot_results(
    results: dict[str, Any],
    gts: list[dict[str, Any]],
    label: str,
    layout: list[tuple[list[str], list[str]]],
    t0: float,
) -> None:
    t_fg = results["time"] - t0

    _, axes = plt.subplots(len(layout), 3, figsize=(15, 8), num=label or None)
    for row, (keys, axis_labels) in enumerate(layout):
        for col, (key, axis_label) in enumerate(zip(keys, axis_labels, strict=True)):
            ax = axes[row, col]
            for gt in gts:
                if key in gt:
                    gt_t, gt_vals = _mask_gaps(gt["time"] - t0, gt[key])
                    ax.plot(gt_t, gt_vals, "-", color=GT_COLOR, label="GT")
            if key in results:
                ax.plot(t_fg, results[key], "-", color=FG_COLOR, label="FG")
            ax.set_ylabel(axis_label)
            if row == len(layout) - 1:
                ax.set_xlabel("Time (s)")

    plt.tight_layout()
