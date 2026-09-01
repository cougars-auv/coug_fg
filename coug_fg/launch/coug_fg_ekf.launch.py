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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def agent_frame(agent_ns: LaunchConfiguration, frame: str) -> PythonExpression:
    return PythonExpression(
        ["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"]
    )


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

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
            PythonExpression(["'", agent_ns, "' + '_params.yaml'"]),
        ]
    )

    odom_frame = agent_frame(agent_ns, "odom")
    base_link_frame = agent_frame(agent_ns, "base_link")
    gps_link_frame = agent_frame(agent_ns, "gps_link")
    depth_link_frame = agent_frame(agent_ns, "depth_link")
    dvl_link_frame = agent_frame(agent_ns, "dvl_link")
    beam0_link_frame = agent_frame(agent_ns, "beam0_link")
    beam1_link_frame = agent_frame(agent_ns, "beam1_link")
    beam2_link_frame = agent_frame(agent_ns, "beam2_link")
    beam3_link_frame = agent_frame(agent_ns, "beam3_link")
    modem_link_frame = agent_frame(agent_ns, "modem_link")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
                description="Namespace for the agent (e.g. auv0)",
            ),
            Node(
                package="coug_fg",
                executable="dvl_a50_twist_beams",
                name="dvl_a50_twist_beams_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "beam0_frame": beam0_link_frame,
                        "beam1_frame": beam1_link_frame,
                        "beam2_frame": beam2_link_frame,
                        "beam3_frame": beam3_link_frame,
                        "parameter_frame": dvl_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="dvl_a50_odom",
                name="dvl_a50_odom_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "parameter_frame": dvl_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="fluid_pressure_odom",
                name="fluid_pressure_odom_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "parameter_child_frame": depth_link_frame,
                    },
                ],
            ),
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
                        "parameter_child_frame": gps_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="seatrac_x150_imu_depth",
                name="seatrac_x150_imu_depth_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "parameter_frame": modem_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="imu_ned_to_enu",
                name="seatrac_imu_ned_to_enu_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="odom_ned_to_enu",
                name="seatrac_odom_ned_to_enu_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="odom_ned_to_enu",
                name="odom_ned_to_enu_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="odom_to_tf",
                name="odom_to_tf_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="gps_to_truth_relay",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
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
    )
