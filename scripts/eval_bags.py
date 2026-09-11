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

import argparse
import logging
import os
from pathlib import Path

import evo_cli
import yaml
from logs import setup_logging

logger = logging.getLogger(__name__)

AGENTS = sorted(
    p.name.removesuffix("_params.yaml")
    for p in Path(os.environ["CONFIG_DIR"]).glob("*_params.yaml")
)
EVO_FLAGS = ["--align"]  # , "--project_to_plane", "xy"]


def _recorded_topics(bag: str) -> set[str]:
    meta = yaml.safe_load((Path(bag) / "metadata.yaml").read_text())
    topics = meta["rosbag2_bagfile_information"]["topics_with_message_count"]
    return {t["topic_metadata"]["name"] for t in topics if t["message_count"]}


def _evaluate_agent(bag: str, agent: str, recorded: set[str], evo_flags: list[str]) -> None:
    agent_dir = evo_cli.evo_agent_dir(bag, agent)
    truth_topic = f"/{agent}/{evo_cli.TRUTH_TOPIC}"
    gt_tum = evo_cli.resolve_tum(bag, agent_dir, truth_topic, recorded)
    if gt_tum is None:
        return

    logger.info(f"Evaluating agent: {agent}")
    for key, est_topic in evo_cli.ESTIMATORS.items():
        out_dir = agent_dir / key
        topic = f"/{agent}/{est_topic}" if est_topic else None
        est_tum = evo_cli.resolve_tum(bag, out_dir, topic, recorded)
        if est_tum is None:
            continue

        logger.info(f"Evaluating estimator: {key}")
        evo_cli.run_evo_evaluations(gt_tum, est_tum, out_dir, evo_flags)

    evo_cli.build_benchmark_tables(agent_dir)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", nargs="+", required=True)
    parser.add_argument("--agents", nargs="+", default=AGENTS)
    parser.add_argument("--evo-flags", default=" ".join(EVO_FLAGS))
    args = parser.parse_args()

    setup_logging()
    evo_flags = args.evo_flags.split()
    for bag in args.bags:
        logger.info(f"Evaluating bag: {bag}")
        recorded = _recorded_topics(bag)
        for agent in args.agents:
            _evaluate_agent(bag, agent, recorded, evo_flags)


if __name__ == "__main__":
    main()
