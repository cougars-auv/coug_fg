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
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

SCRIPTS_PATH = str(Path(__file__).resolve().parents[1] / "scripts")
sys.path.insert(0, SCRIPTS_PATH)

import evo_cli
import run_offline_fg
from offline import graph
from offline.extractors import EXTRACTORS

NAMESPACE = "tank"
BAGS = [
    Path(os.environ["BAGS_DIR"]) / "tank/tank_ros2_beam_loss" / f"{seq}_ros2_beam_loss"
    for seq in ("Structure_Medium", "Structure_Hard")
]
PARAMS_DIR = Path(__file__).parent / "params"
VARIANTS = sorted(p.stem for p in PARAMS_DIR.glob("*.yaml"))
DROPOUTS = [(0.0, 0), *itertools.product(np.arange(1, 9) / 10, range(10))]

_add_message = graph.OfflineFactorGraph.add_message


def _load_pings(bag: Path) -> dict[float, tuple[Any, ...]]:
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        conns = [c for c in reader.connections if c.topic == f"/{NAMESPACE}/dvl/beams"]
        msgs = (reader.deserialize(rawdata, c.msgtype) for c, _, rawdata in reader.messages(conns))
        return {
            measurement[0]: (frame_ids, measurement)
            for frame_ids, measurement in map(EXTRACTORS["beams"], msgs)
        }


def _patch_dropout(prob: float, seed: int, pings: dict[float, tuple[Any, ...]]) -> None:
    def add_message(
        self: graph.OfflineFactorGraph,
        sensor: str,
        frame_id: str | list[str],
        measurement: tuple[Any, ...],
        log_time: float,
    ) -> None:
        stamp = measurement[0]
        rng = np.random.default_rng([seed, round(stamp * 1e6)])
        dropped = rng.random(graph.NUM_DVL_BEAMS) < prob
        if sensor == "beams":
            beams = [(*b[:3], b[3] and not d) for b, d in zip(measurement[1], dropped, strict=True)]
            measurement = (stamp, beams)
        elif sensor == "dvl" and stamp in pings:
            frame_ids, (_, beams) = pings[stamp]
            kept = [i for i, b in enumerate(beams) if b[3] and not dropped[i]]
            if len(kept) < 3:
                return
            assert self._urdf is not None and isinstance(frame_id, str)
            quats = [self._urdf.lookup(frame_id, frame_ids[i])[1] for i in kept]
            dirs = Rotation.from_quat(quats).apply([1, 0, 0])
            velocity = np.linalg.lstsq(dirs, [beams[i][1] for i in kept], rcond=None)[0]
            measurement = (stamp, velocity, measurement[2])
        _add_message(self, sensor, frame_id, measurement, log_time)

    graph.OfflineFactorGraph.add_message = add_message  # type: ignore[method-assign, unused-ignore]


def main() -> None:
    for bag in BAGS:
        pings = _load_pings(bag)
        for (prob, seed), variant in itertools.product(DROPOUTS, VARIANTS):
            prefix = f"beam_loss_{variant}_p{round(prob * 100)}_s{seed}"
            if (evo_cli.evo_agent_dir(bag, NAMESPACE) / prefix / "ape_trans.zip").exists():
                continue

            _patch_dropout(prob, seed, pings)
            sys.argv = [
                sys.argv[0],
                "--bags",
                str(bag),
                "--namespace",
                NAMESPACE,
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
