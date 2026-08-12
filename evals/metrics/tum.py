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
from collections.abc import Generator
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)

TUM_KEYS = ("time", "x", "y", "z", "qx", "qy", "qz", "qw")


def evo_agent_dir(bag_path: str | Path, namespace: str) -> Path:
    """
    Return the bag's evo output directory for one agent namespace.

    :param bag_path: Path to the ROS 2 bag directory.
    :param namespace: AUV namespace whose outputs are stored.
    :return: The agent's evo output directory.
    """
    return Path(bag_path) / "evo" / namespace


def iter_evaluated_agents(target_dir: Path) -> Generator[tuple[Path, Path]]:
    """
    Yield each evaluated agent directory under a target directory.

    :param target_dir: A bag or directory of bags that has been evaluated.
    :return: ``(bag_dir, agent_dir)`` pairs found under every ``evo`` folder.
    """
    for evo_dir in target_dir.rglob("evo"):
        bag_dir = evo_dir.parent
        if not (bag_dir / "metadata.yaml").exists():
            continue
        for agent_dir in filter(Path.is_dir, evo_dir.iterdir()):
            yield bag_dir, agent_dir


def latest_tum(directory: Path) -> Path | None:
    """
    Return the most recently modified TUM file in a directory, if any.

    :param directory: Directory to search for ``*.tum`` files.
    :return: The newest TUM file by modification time, or None if none exist.
    """
    tum_files = sorted(directory.glob("*.tum"), key=lambda p: p.stat().st_mtime)
    return tum_files[-1] if tum_files else None


def load_tum(path: Path) -> dict:
    """
    Read a TUM-format text file into state arrays.

    :param path: TUM file to read.
    :return: Arrays keyed by state name (TUM_KEYS plus euler angles), or empty on error.
    """
    try:
        data = np.loadtxt(path, comments="#", ndmin=2)
    except (OSError, ValueError):
        logger.error(f"Could not load TUM trajectory from {path}")
        return {}
    if data.size == 0:
        logger.error(f"TUM file is empty: {path}")
        return {}
    if data.shape[1] < len(TUM_KEYS):
        logger.error(f"TUM file has too few columns: {path}")
        return {}

    pose = {k: data[:, i] for i, k in enumerate(TUM_KEYS)}
    roll, pitch, yaw = Rotation.from_quat(data[:, 4:8]).as_euler("xyz").T
    pose.update({"roll": roll, "pitch": pitch, "yaw": yaw})
    return pose


def save_tum(path: Path, results: dict) -> None:
    """
    Write trajectory arrays to a TUM-format text file.

    :param path: Destination file path.
    :param results: Arrays keyed by state name (must contain TUM_KEYS).
    """
    np.savetxt(
        path,
        np.column_stack([results[k] for k in TUM_KEYS]),
        fmt="%.9f",
    )
    logger.info(f"Saved TUM trajectory: {path}")
