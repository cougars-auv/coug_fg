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

from typing import Any

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Node]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_list = yaml.safe_load(agent_list_str)
    agent_ns = agent_list[0]

    fleet_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_fg_params.yaml",
        ]
    )
    agent_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            f"{agent_ns}_params.yaml",
        ]
    )

    return [
        Node(
            package="coug_fg",
            executable="navsat_odom",
            name="navsat_odom_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "set_origin": True,
                    "input_topic": f"/{agent_ns}/gps/fix",
                    "output_topic": "gps/odometry_null",
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
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of agent namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
