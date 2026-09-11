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

import logging
import subprocess
from collections import defaultdict
from operator import attrgetter
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)

TRUTH_TOPIC = "odometry/truth"
SIM_TRUTH_FIELDS = {
    "VelocitySensor": {"v": "twist.twist.linear"},
    "imu/bias": {"accel_bias_": "twist.twist.linear", "gyro_bias_": "twist.twist.angular"},
    "imu/mag/bias": {"mag_bias_": "magnetic_field"},
}
ESTIMATORS: dict[str, str] = {
    "global": "odometry/global",
    "global_isam2": "odometry/global_isam2",
    "global_lpi": "odometry/global_lpi",
    "global_tpi": "odometry/global_tpi",
    "global_tm": "",
    "global_iekf": "odometry/global_iekf",
    "global_ukf": "odometry/global_ukf",
    "global_ekf": "odometry/global_ekf",
    "global_nbr": "base/odometry/global_nbr",
    "imu": "imu/odometry",
    "dvl": "dvl/odometry",
}

TUM_KEYS = ("time", "x", "y", "z", "qx", "qy", "qz", "qw")
BASE_FLAGS = ["--t_max_diff", "0.05", "--no_warnings"]
RPE_FLAGS = [
    "--delta",
    "1",
    "--delta_unit",
    "m",
    "--all_pairs",
    "--pairs_from_reference",
]
EVO_RUNS = (
    ("evo_ape", "trans_part", "ape_trans"),
    ("evo_ape", "angle_deg", "ape_rot"),
    ("evo_rpe", "trans_part", "rpe_trans"),
    ("evo_rpe", "angle_deg", "rpe_rot"),
)


def evo_agent_dir(bag_path: str | Path, namespace: str) -> Path:
    return Path(bag_path) / "evo" / namespace


def _latest_tum(directory: Path) -> Path | None:
    return max(directory.glob("*.tum"), key=lambda p: p.stat().st_mtime, default=None)


def save_tum(path: Path, pose: dict[str, Any]) -> None:
    np.savetxt(path, np.column_stack([pose[k] for k in TUM_KEYS]), fmt="%.9f")
    logger.info(f"Saved TUM trajectory: {path}")


def _load_tum(path: Path) -> dict[str, Any]:
    data = np.loadtxt(path, ndmin=2)
    pose = {k: data[:, i] for i, k in enumerate(TUM_KEYS)}
    pose["roll"], pose["pitch"], pose["yaw"] = Rotation.from_quat(data[:, 4:8]).as_euler("xyz").T
    return pose


def _export_bag_tum(bag_path: str | Path, out_dir: Path, topic: str) -> Path | None:
    out_dir.mkdir(parents=True, exist_ok=True)
    args = ["evo_traj", "bag2", str(Path(bag_path).resolve()), topic, "--save_as_tum"]
    code = subprocess.run(args, cwd=out_dir, check=False).returncode
    if code != 0:
        return None

    return _latest_tum(out_dir)


def resolve_tum(
    bag_path: str | Path,
    out_dir: Path,
    topic: str | None,
    recorded: set[str] | None = None,
) -> Path | None:
    tum = _latest_tum(out_dir)
    if tum is not None:
        return tum
    if not topic or (recorded is not None and topic not in recorded):
        return None
    return _export_bag_tum(bag_path, out_dir, topic)


def load_ground_truth(bag_path: str | Path, namespace: str) -> tuple[dict[str, Any], Path | None]:
    agent_dir = evo_agent_dir(bag_path, namespace)
    truth_topic = f"/{namespace}/{TRUTH_TOPIC}"
    tum_path = resolve_tum(bag_path, agent_dir, truth_topic)
    if tum_path is None:
        return {}, None

    return _load_tum(tum_path), tum_path


def load_sim_ground_truth(bag_path: str | Path, namespace: str) -> list[dict[str, Any]]:
    fields = {f"/{namespace}/{topic}": f for topic, f in SIM_TRUTH_FIELDS.items()}
    gts: dict[str, dict[str, list[float]]] = {topic: defaultdict(list) for topic in fields}
    with AnyReader([Path(bag_path)], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        conns = [c for c in reader.connections if c.topic in fields]
        for conn, _, rawdata in reader.messages(connections=conns) if conns else ():
            msg: Any = reader.deserialize(rawdata, conn.msgtype)
            gt = gts[conn.topic]
            gt["time"].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            for key, field in fields[conn.topic].items():
                for axis in "xyz":
                    gt[key + axis].append(attrgetter(f"{field}.{axis}")(msg))

    return [{k: np.array(v) for k, v in gt.items()} for gt in gts.values() if gt]


def run_evo_evaluations(
    gt_file: str | Path, est_file: str | Path, evo_dir: Path, evo_flags: list[str]
) -> None:
    for cmd, pose_relation, name in EVO_RUNS:
        archive = evo_dir / f"{name}.zip"
        archive.unlink(missing_ok=True)
        args = [cmd, "tum", str(gt_file), str(est_file), "-r", pose_relation]
        args += BASE_FLAGS + evo_flags + ["--save_results", str(archive)]
        if cmd == "evo_rpe":
            args += RPE_FLAGS

        subprocess.run(args, check=False)


def build_benchmark_tables(agent_dir: Path) -> None:
    for old_table in agent_dir.glob("benchmark_*.csv"):
        old_table.unlink()

    for _, _, name in EVO_RUNS:
        archives = sorted(agent_dir.glob(f"*/{name}.zip"))
        if not archives:
            continue

        table = agent_dir / f"benchmark_{name}.csv"
        args = [
            "evo_res",
            *map(str, archives),
            "--no_warnings",
            "--save_table",
            str(table),
        ]
        subprocess.run(args, check=False)
