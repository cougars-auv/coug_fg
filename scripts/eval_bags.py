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
from pathlib import Path

import evo_cli
import yaml
from logs import setup_logging

logger = logging.getLogger(__name__)

TARGET_DIR = Path(os.environ["BAGS_DIR"])
AGENTS = sorted(
    p.name.removesuffix("_params.yaml")
    for p in Path(os.environ["CONFIG_DIR"]).glob("*_params.yaml")
)
EVO_FLAGS = ["--align"]  # , "--project_to_plane", "xy"]


def _recorded_topics(bag_path: Path) -> set[str]:
    meta = yaml.safe_load((bag_path / "metadata.yaml").read_text())
    topics = meta["rosbag2_bagfile_information"]["topics_with_message_count"]
    return {t["topic_metadata"]["name"] for t in topics if t["message_count"]}


def _evaluate_agent(
    bag_path: Path, agent: str, recorded: set[str], evo_flags: list[str]
) -> None:
    agent_dir = evo_cli.evo_agent_dir(bag_path, agent)
    truth_topic = f"/{agent}/{evo_cli.TRUTH_TOPIC}"
    gt_tum = evo_cli.resolve_tum(bag_path, agent_dir, truth_topic, recorded)
    if gt_tum is None:
        return

    logger.info(f"Evaluating agent: {agent}")
    for key, est_topic in evo_cli.ESTIMATORS.items():
        out_dir = agent_dir / key
        topic = f"/{agent}/{est_topic}" if est_topic else None
        est_tum = evo_cli.resolve_tum(bag_path, out_dir, topic, recorded)
        if est_tum is None:
            continue

        logger.info(f"Evaluating estimator: {key}")
        evo_cli.run_evo_evaluations(gt_tum, est_tum, out_dir, evo_flags)

    evo_cli.build_benchmark_tables(agent_dir)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-dir", type=Path, default=TARGET_DIR)
    parser.add_argument("--agents", nargs="+", default=AGENTS)
    parser.add_argument("--evo-flags", default=" ".join(EVO_FLAGS))
    args = parser.parse_args()

    setup_logging()
    evo_flags = args.evo_flags.split()
    bags = sorted(meta.parent for meta in args.target_dir.rglob("metadata.yaml"))
    if not bags:
        logger.error(f"No bags found in {args.target_dir}")
        return

    for bag_path in bags:
        logger.info(f"Evaluating bag: {bag_path}")
        recorded = _recorded_topics(bag_path)
        for agent in args.agents:
            _evaluate_agent(bag_path, agent, recorded, evo_flags)


if __name__ == "__main__":
    main()
