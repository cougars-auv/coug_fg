# Copyright (c) 2026 BYU FROST Lab
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

import matplotlib.pyplot as plt
import numpy as np

GT_COLOR = "#000000"
FGO_COLOR = "#55A868"

LAYOUT = [
    (["x", "y", "z"], ["X (m)", "Y (m)", "Z (m)"], True),
    (["roll", "pitch", "yaw"], ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"], True),
    (["vx", "vy", "vz"], ["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"], False),
    (
        ["bias_accel_x", "bias_accel_y", "bias_accel_z"],
        ["Accel Bias X", "Accel Bias Y", "Accel Bias Z"],
        False,
    ),
    (
        ["bias_gyro_x", "bias_gyro_y", "bias_gyro_z"],
        ["Gyro Bias X", "Gyro Bias Y", "Gyro Bias Z"],
        False,
    ),
    (
        ["bias_mag_x", "bias_mag_y", "bias_mag_z"],
        ["Mag Bias X (T)", "Mag Bias Y (T)", "Mag Bias Z (T)"],
        False,
    ),
]


def _mask_gaps(t: np.ndarray, vals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(t) < 2:
        return t, vals
    dts = np.diff(t)
    gaps = np.where(dts > max(float(np.median(dts)) * 5.0, 0.5))[0] + 1
    if not len(gaps):
        return t, vals
    return np.insert(t, gaps, np.nan), np.insert(vals, gaps, np.nan)


def plot_results(results: dict, pose_gt: dict, label: str = "") -> None:
    t0 = results["time"][0]
    t_fgo = results["time"] - t0

    _, axes = plt.subplots(len(LAYOUT), 3, figsize=(15, 8), num=label or None)
    for row, (keys, axis_labels, show_gt) in enumerate(LAYOUT):
        for col, (key, axis_label) in enumerate(zip(keys, axis_labels)):
            ax = axes[row, col]
            if show_gt and pose_gt:
                gt_t, gt_vals = _mask_gaps(pose_gt["time"] - t0, pose_gt[key])
                ax.plot(gt_t, gt_vals, "-", color=GT_COLOR, label="GT")
            if key in results:
                ax.plot(t_fgo, results[key], "-", color=FGO_COLOR, label="FGO")
            ax.set_ylabel(axis_label)
            if row == len(LAYOUT) - 1:
                ax.set_xlabel("Time (s)")

    plt.tight_layout()
