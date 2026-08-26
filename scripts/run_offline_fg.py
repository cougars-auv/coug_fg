#!/usr/bin/env python3
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

import argparse
import logging
import os
import shutil
from pathlib import Path

import evo_cli
import matplotlib.pyplot as plt
import state_plot
from logs import setup_logging
from offline import pipeline
from tqdm.contrib.logging import logging_redirect_tqdm

logger = logging.getLogger(__name__)

NAMESPACE = "turtlmap"
EVO_FLAGS = ["--align"]  # , "--project_to_plane", "xy"]


def _config_paths(namespace: str) -> list[str]:
    config_dir = Path(os.environ["CONFIG_DIR"])
    return [
        str(config_dir / "fleet" / "coug_fg_params.yaml"),
        str(config_dir / f"{namespace}_params.yaml"),
    ]


def _save_config(dest_dir: Path) -> None:
    config_dir = os.environ.get("CONFIG_DIR", "")
    if not config_dir or not Path(config_dir).is_dir():
        return

    dest = dest_dir / "config"
    shutil.copytree(config_dir, dest, dirs_exist_ok=True)
    logger.info(f"Config saved: {dest}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", nargs="+", required=True)
    parser.add_argument("--namespace", default=NAMESPACE)
    parser.add_argument("--tag", default="offline")
    parser.add_argument("--evo-flags", default=" ".join(EVO_FLAGS))
    args = parser.parse_args()

    setup_logging()
    evo_flags = args.evo_flags.split()
    cfg_paths = _config_paths(args.namespace)
    plot_args = []

    with logging_redirect_tqdm():
        for bag in args.bags:
            logger.info(f"Processing bag: {bag}")
            pose_gt, gt_path = evo_cli.load_ground_truth(bag, args.namespace)
            results = pipeline.process_bag_offline(bag, cfg_paths, args.namespace)
            if not results:
                continue

            evo_dir = evo_cli.evo_agent_dir(bag, args.namespace) / args.tag
            evo_dir.mkdir(parents=True, exist_ok=True)
            _save_config(evo_dir)
            est_path = evo_dir / f"{args.namespace}_{args.tag}.tum"
            evo_cli.save_tum(est_path, results)

            if gt_path is not None:
                evo_cli.run_evo_evaluations(gt_path, est_path, evo_dir, evo_flags)

            plot_args.append((results, pose_gt, Path(bag).name))

    for results, pose_gt, label in plot_args:
        state_plot.plot_results(results, pose_gt, label)
    plt.show()


if __name__ == "__main__":
    main()
