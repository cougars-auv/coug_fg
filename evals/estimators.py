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

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Estimator:
    key: str
    label: str
    color: str
    topic: str | None = None
    node: str | None = None


TRUTH_TOPIC = "odometry/truth"
GROUND_TRUTH_COLOR = "#000000"

ESTIMATORS: list[Estimator] = [
    Estimator(
        "global", "FL-B", "#55A868", topic="odometry/global", node="factor_graph_node"
    ),
    Estimator(
        "global_isam2",
        "iS2-B",
        "#DD8452",
        topic="odometry/global_isam2",
        node="factor_graph_node_isam2",
    ),
    Estimator(
        "global_lpi",
        "FL-LPI",
        "#4C72B0",
        topic="odometry/global_lpi",
        node="factor_graph_node_lpi",
    ),
    Estimator(
        "global_tpi",
        "FL-TPI",
        "#C44E52",
        topic="odometry/global_tpi",
        node="factor_graph_node_tpi",
    ),
    Estimator("global_iekf", "IEKF", "#8172B3", topic="odometry/global_iekf"),
    Estimator("global_ukf", "UKF", "#937860", topic="odometry/global_ukf"),
    Estimator("global_ekf", "EKF", "#DA8BC3", topic="odometry/global_ekf"),
    Estimator("global_tm", "TM", "#8C8C8C"),
    Estimator("imu", "SBG", "#CCB974", topic="imu/odometry"),
    Estimator("dvl", "DVL", "#64B5CD", topic="dvl/odometry"),
]


def timed_estimators() -> list[Estimator]:
    return [e for e in ESTIMATORS if e.node is not None]


def label_for_folder(folder: str) -> str | None:
    return next((e.label for e in ESTIMATORS if e.key == folder), None)


def label_for_row(row_key: str) -> str | None:
    stem = Path(str(row_key)).stem
    tokens = [
        (e.topic.replace("/", "_") if e.topic else e.key, e.label) for e in ESTIMATORS
    ]
    for token, label in sorted(tokens, key=lambda t: len(t[0]), reverse=True):
        if stem == token or stem.endswith(f"_{token}"):
            return label
    return None


def color_map() -> dict[str, str]:
    return {e.label: e.color for e in ESTIMATORS} | {"GT": GROUND_TRUTH_COLOR}


def labels() -> list[str]:
    return [e.label for e in ESTIMATORS]
