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

import os
from typing import Any

import yaml
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["coug_fg_base_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")

    agent_list_str = LaunchConfiguration("agent_list").perform(context)
    scenario_param_path = LaunchConfiguration("scenario_param_file").perform(context)

    agent_list = yaml.safe_load(agent_list_str)
    agent_ns = agent_list[0]

    config_dir = os.environ["CONFIG_DIR"]

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_fg_params.yaml"]
    )
    scenario_param_file = scenario_param_path or fleet_param_file

    fleet_param_path = os.path.join(config_dir, "fleet", "coug_fg_params.yaml")
    agent_param_path = os.path.join(config_dir, f"{agent_ns}_params.yaml")

    launch_params = {
        **load_launch_params(fleet_param_path, "/**"),
        **load_launch_params(agent_param_path, f"/{agent_ns}"),
        **load_launch_params(scenario_param_path, "/**"),
        **load_launch_params(scenario_param_path, f"/{agent_ns}"),
    }
    gps_topic_name = launch_params["gps_topic"]
    gps_topic = (
        gps_topic_name if gps_topic_name.startswith("/") else f"/{agent_ns}/{gps_topic_name}"
    )

    return [
        Node(
            package="coug_fg",
            executable="navsat_odom",
            name="navsat_odom_node",
            parameters=[
                fleet_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "input_topic": gps_topic,
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
