#!/usr/bin/env python3
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

import itertools
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt

SCRIPTS_PATH = str(Path(__file__).resolve().parents[1] / "scripts")
sys.path.insert(0, SCRIPTS_PATH)

import evo_cli
import run_offline_fg

BAGS = {
    namespace: sorted(Path(os.environ["BAGS_DIR"]).glob(pattern))
    for namespace, pattern in (
        ("bluerov2", "bluerov2/utah_lake_accuracy/*_accuracy"),
        ("tank", "tank/tank_ros2_accuracy/[HS]*_accuracy"),
        ("tankwhole", "tank/tank_ros2_accuracy/W*_accuracy"),
        ("turtlmap", "turtlmap/turtlmap_ros2_accuracy/*_accuracy"),
    )
}
PARAMS_DIR = Path(__file__).parent / "params"
VARIANTS = sorted(p.stem for p in PARAMS_DIR.glob("*.yaml"))


def main() -> None:
    for (namespace, bags), variant in itertools.product(BAGS.items(), VARIANTS):
        for bag in bags:
            prefix = f"accuracy_{variant}"
            if (evo_cli.evo_agent_dir(bag, namespace) / prefix / "ape_trans.zip").exists():
                continue

            sys.argv = [
                sys.argv[0],
                "--bags",
                str(bag),
                "--namespace",
                namespace,
                "--prefix",
                prefix,
                "--override",
                str(PARAMS_DIR / f"{variant}.yaml"),
                "--no-show",
            ]
            run_offline_fg.main()
            plt.close("all")


if __name__ == "__main__":
    main()
