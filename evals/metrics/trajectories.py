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

import logging

import numpy as np
from evo.core import lie_algebra as lie
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


def dict_to_trajectory(pose: dict) -> PoseTrajectory3D:
    return PoseTrajectory3D(
        positions_xyz=np.column_stack([pose["x"], pose["y"], pose["z"]]),
        orientations_quat_wxyz=np.column_stack(
            [pose["qw"], pose["qx"], pose["qy"], pose["qz"]]
        ),
        timestamps=pose["time"],
    )


def compute_ape_rmse(
    gt: dict | None, est: dict | None, crashed: bool = False, max_diff: float = 0.05
) -> float:
    if not gt or not est or crashed:
        return float("inf")

    try:
        gt_sync, est_sync = sync.associate_trajectories(
            dict_to_trajectory(gt), dict_to_trajectory(est), max_diff=max_diff
        )
        est_sync.align(gt_sync)

        ape = metrics.APE(metrics.PoseRelation.translation_part)
        ape.process_data((gt_sync, est_sync))
        return ape.get_statistic(metrics.StatisticsType.rmse)
    except Exception as e:  # noqa: BLE001
        logger.error(f"Could not compute APE RMSE: {e}")
        return float("inf")


def umeyama_align(est: PoseTrajectory3D, ref: PoseTrajectory3D) -> None:
    ref_sync, est_sync = sync.associate_trajectories(ref, est, max_diff=0.05)
    if est_sync.num_poses < 2:
        return
    r, t, s = est_sync.align(ref_sync, correct_scale=False)
    est.scale(s)
    est.transform(lie.se3(r, t))


def align_dicts(est: dict, ref: dict) -> None:
    est_traj = dict_to_trajectory(est)
    umeyama_align(est_traj, dict_to_trajectory(ref))

    for i, key in enumerate(("x", "y", "z")):
        est[key] = est_traj.positions_xyz[:, i]
    for i, key in enumerate(("qw", "qx", "qy", "qz")):
        est[key] = est_traj.orientations_quat_wxyz[:, i]

    quats_xyzw = est_traj.orientations_quat_wxyz[:, [1, 2, 3, 0]]
    est["roll"], est["pitch"], est["yaw"] = (
        Rotation.from_quat(quats_xyzw).as_euler("xyz").T
    )
