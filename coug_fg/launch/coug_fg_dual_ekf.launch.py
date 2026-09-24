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

import json
from typing import Any

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitution import Substitution
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def agent_frame(agent_ns: str | Substitution, frame: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"])


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

    initial_position_str = LaunchConfiguration("initial_position").perform(context)
    initial_orientation_str = LaunchConfiguration("initial_orientation").perform(context)
    scenario_param_path = LaunchConfiguration("scenario_param_file").perform(context)

    position = json.loads(initial_position_str) if initial_position_str else None
    orientation = json.loads(initial_orientation_str) if initial_orientation_str else None

    initial_state_params: list[dict[str, Any]] = []
    if position is not None and orientation is not None:
        initial_state_params = [{"initial_state": position + orientation + [0.0] * 9}]

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_fg_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = scenario_param_path or agent_param_file

    odom_frame = agent_frame(agent_ns, "odom")
    base_link_frame = agent_frame(agent_ns, "base_link")
    gps_link_frame = agent_frame(agent_ns, "gps_link")

    return [
        Node(
            package="coug_fg",
            executable="navsat_odom",
            name="navsat_odom_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "parameter_child_frame": gps_link_frame,
                },
            ],
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("imu/data_raw", "camera/imu/data_raw"),
                ("imu/mag", "camera/imu/mag"),
                ("imu/data", "camera/imu/data"),
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "odom_frame": odom_frame,
                    "base_link_frame": base_link_frame,
                    "world_frame": odom_frame,
                },
            ],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            parameters=[
                fleet_param_file,
                *initial_state_params,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "odom_frame": odom_frame,
                    "base_link_frame": base_link_frame,
                    "world_frame": "map",
                },
            ],
            remappings=[("odometry/filtered", "odometry/global")],
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
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "initial_position",
                default_value="",
            ),
            DeclareLaunchArgument(
                "initial_orientation",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
