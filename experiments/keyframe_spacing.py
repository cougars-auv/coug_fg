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

import matplotlib.pyplot as plt
import yaml

SCRIPTS_PATH = str(Path(__file__).resolve().parents[1] / "scripts")
sys.path.insert(0, SCRIPTS_PATH)

import evo_cli
import run_offline_fg
from offline import graph

NAMESPACE = "tank"
BAGS = [
    Path(os.environ["BAGS_DIR"])
    / "tank/tank_ros2_keyframe_spacing"
    / f"{seq}_ros2_keyframe_spacing"
    for seq in ("Structure_Easy", "Structure_Medium")
]
PARAMS_DIR = Path(__file__).parent / "params"
VARIANTS = sorted(p.stem for p in PARAMS_DIR.glob("*.yaml"))
SPACINGS = (0.02, 0.25, 0.5, 1.0, 2.0, 4.0)
OFFSETS = range(5)

_update_graph = graph.OfflineFactorGraph._update_graph


def _patch_offset(spacing: float, offset: int) -> None:
    anchor = None

    def update_graph(self: graph.OfflineFactorGraph) -> None:
        nonlocal anchor
        last = self._last_target_time
        _update_graph(self)
        if self._last_target_time is None or self._last_target_time == last:
            return
        if anchor is None:
            anchor = self._last_target_time - offset / len(OFFSETS) * spacing
        self._last_target_time = anchor + (self._last_target_time - anchor) // spacing * spacing

    graph.OfflineFactorGraph._update_graph = update_graph  # type: ignore[method-assign, unused-ignore]


def main() -> None:
    runs = [(SPACINGS[0], 0), *itertools.product(SPACINGS[1:], OFFSETS)]
    for bag, (spacing, offset), variant in itertools.product(BAGS, runs, VARIANTS):
        prefix = f"keyframe_spacing_{variant}_{round(spacing * 1000)}ms_o{offset}"
        if (evo_cli.evo_agent_dir(bag, NAMESPACE) / prefix / "ape_trans.zip").exists():
            continue

        _patch_offset(spacing, offset)
        params = yaml.safe_load((PARAMS_DIR / f"{variant}.yaml").read_text())
        params["/**"]["factor_graph_node"]["ros__parameters"]["min_keyframe_interval_sec"] = spacing
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as override:
            yaml.safe_dump(params, override)
            override.flush()
            sys.argv = [
                sys.argv[0],
                "--bags",
                str(bag),
                "--namespace",
                NAMESPACE,
                "--prefix",
                prefix,
                "--override",
                override.name,
                "--no-show",
            ]
            run_offline_fg.main()
            plt.close("all")


if __name__ == "__main__":
    main()
