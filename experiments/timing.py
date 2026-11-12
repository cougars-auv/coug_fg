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
import tempfile
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml

SCRIPTS_PATH = str(Path(__file__).resolve().parents[1] / "scripts")
sys.path.insert(0, SCRIPTS_PATH)

import evo_cli
import run_offline_fg
from offline import pipeline

BAGS = {
    "bluerov2": Path(os.environ["BAGS_DIR"])
    / "bluerov2/utah_lake_timing/ul_surface_1.0-2026-01-23-11-06-12_timing",
    "turtlmap": Path(os.environ["BAGS_DIR"]) / "turtlmap/turtlmap_ros2_timing/log1_ros2_timing",
}
PARAMS_DIR = Path(__file__).parent / "params"
VARIANTS = sorted(p.stem for p in PARAMS_DIR.glob("*.yaml"))
LAGS = (5, 10, 20, 40, 60)
DURATIONS = ("total_duration", "smoother_duration", "cov_duration")

_process_bag_offline = pipeline.process_bag_offline


def _patch_results(results: list[Any]) -> None:
    def process_bag_offline(
        bag_path: str, config_paths: list[str], namespace: str
    ) -> dict[str, Any] | None:
        result: dict[str, Any] | None = _process_bag_offline(bag_path, config_paths, namespace)
        results.append(result)
        return result

    pipeline.process_bag_offline = process_bag_offline


def main() -> None:
    results: list[Any] = []
    _patch_results(results)
    for (namespace, bag), lag, variant in itertools.product(BAGS.items(), LAGS, VARIANTS):
        prefix = f"timing_{variant}_lag{lag}"
        times_path = evo_cli.evo_agent_dir(bag, namespace) / prefix / "update_times.csv"
        if times_path.exists():
            continue

        params = yaml.safe_load((PARAMS_DIR / f"{variant}.yaml").read_text())
        params["/**"]["factor_graph_node"]["ros__parameters"]["smoother_lag_sec"] = float(lag)
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as override:
            yaml.safe_dump(params, override)
            override.flush()
            sys.argv = [
                sys.argv[0],
                "--bags",
                str(bag),
                "--namespace",
                namespace,
                "--prefix",
                prefix,
                "--override",
                override.name,
                "--no-show",
            ]
            run_offline_fg.main()
            plt.close("all")
        result = results.pop()
        np.savetxt(
            times_path,
            np.column_stack([result[key] for key in DURATIONS]),
            delimiter=",",
            header=",".join(f"{key} [s]" for key in DURATIONS),
        )


if __name__ == "__main__":
    main()
