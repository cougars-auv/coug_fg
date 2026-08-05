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
import subprocess
from pathlib import Path

import estimators

from metrics import tum

logger = logging.getLogger(__name__)


def run_logged(args: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess:
    """
    Run a subprocess. It will write directly to the terminal.

    :param args: Command and arguments to execute.
    :param cwd: Working directory to run the command in, if any.
    :return: The completed process.
    """
    return subprocess.run(args, cwd=cwd, check=False)


def export_bag_tum(bag_path: str | Path, topic: str, out_dir: Path) -> Path | None:
    """
    Export a recorded trajectory topic from a bag to a TUM file with evo.

    :param bag_path: Path to the ROS 2 bag directory.
    :param topic: Trajectory topic to export (e.g. an odometry topic).
    :param out_dir: Directory to write the exported TUM file into.
    :return: Path to the exported TUM file, or None if the export failed.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    args = ["evo_traj", "bag2", str(Path(bag_path).resolve()), topic, "--save_as_tum"]
    if run_logged(args, cwd=out_dir).returncode != 0:
        return None

    return tum.latest_tum(out_dir)


def ensure_ground_truth(bag_path: str | Path, namespace: str) -> Path | None:
    """
    Return the agent's ground truth TUM file, exporting it from the bag if needed.

    :param bag_path: Path to the ROS 2 bag directory.
    :param namespace: AUV namespace the ground truth belongs to.
    :return: Path to the ground truth TUM file, or None if it could not be produced.
    """
    agent_dir = tum.evo_agent_dir(bag_path, namespace)
    tum_path = tum.latest_tum(agent_dir)
    if tum_path is None:
        logger.warning(
            f"No ground truth TUM found in {agent_dir}; attempting export..."
        )
        truth_topic = f"/{namespace}/{estimators.TRUTH_TOPIC}"
        tum_path = export_bag_tum(bag_path, truth_topic, agent_dir)
    return tum_path


def load_ground_truth(bag_path: str | Path, namespace: str) -> tuple[dict, Path | None]:
    """
    Load the ground truth into state arrays, exporting it from the bag first if needed.

    :param bag_path: Path to the ROS 2 bag directory.
    :param namespace: AUV namespace the ground truth was exported under.
    :return: Tuple of arrays keyed by state name, and the path to the TUM file.
    """
    tum_path = ensure_ground_truth(bag_path, namespace)
    if tum_path is None:
        logger.error(f"Could not find or export ground truth for {namespace}.")
        return {}, None

    pose = tum.load_tum(tum_path)
    if not pose:
        return {}, None

    logger.info(f"Loaded ground truth: {tum_path}")
    return pose, tum_path


def run_evo_evaluations(
    gt_file: str | Path, est_file: str | Path, evo_dir: Path, evo_flags: list[str]
) -> None:
    """
    Run the evo APE and RPE evaluations and save the result archives.

    :param gt_file: Ground truth trajectory in TUM format.
    :param est_file: Estimated trajectory in TUM format.
    :param evo_dir: Directory to save the evo result archives in.
    :param evo_flags: Extra evo flags forwarded to every APE and RPE run.
    """
    base_flags = ["--t_max_diff", "0.05", "--no_warnings"]

    for metric, cmd in [("APE", "evo_ape"), ("RPE", "evo_rpe")]:
        for pose_relation, suffix in [("trans_part", "trans"), ("angle_deg", "rot")]:
            args = [cmd, "tum", str(gt_file), str(est_file), "-r", pose_relation]
            args += base_flags + evo_flags
            args += ["--save_results", str(evo_dir / f"{metric.lower()}_{suffix}.zip")]
            if metric == "RPE":
                args += ["--delta", "1", "--delta_unit", "m", "--all_pairs"]

            run_logged(args)


def build_benchmark_tables(agent_dir: Path, metrics_names: tuple[str, ...]) -> None:
    """
    Aggregate an agent's evo result archives into per-metric benchmark tables.

    :param agent_dir: The agent's evo output directory holding the result zips.
    :param metrics_names: Metric names to tabulate (e.g. ``ape_trans``).
    """
    if not any(agent_dir.glob("*/*.zip")):
        return

    for old_table in agent_dir.glob("benchmark_*.csv"):
        old_table.unlink()

    for metric in metrics_names:
        metric_zips = sorted(
            zip_path
            for est in estimators.ESTIMATORS
            if (zip_path := agent_dir / est.key / f"{metric}.zip").exists()
        )
        if not metric_zips:
            continue
        args = ["evo_res", *map(str, metric_zips)]
        args += ["--save_table", str(agent_dir / f"benchmark_{metric}.csv")]
        run_logged(args)
