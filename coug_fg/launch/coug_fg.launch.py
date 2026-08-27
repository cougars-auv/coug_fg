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
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")
    loc_comparison = LaunchConfiguration("loc_comparison")
    lead_agent = LaunchConfiguration("lead_agent")

    is_lead_agent = EqualsSubstitution(agent_ns, lead_agent)

    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_fg_params.yaml",
        ]
    )
    agent_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            PythonExpression(["'", agent_ns, "' + '_params.yaml'"]),
        ]
    )

    odom_frame = PythonExpression(
        ["'", agent_ns, "/odom' if '", agent_ns, "' != '' else 'odom'"]
    )

    base_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/base_link' if '",
            agent_ns,
            "' != '' else 'base_link'",
        ]
    )

    imu_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/imu_link' if '",
            agent_ns,
            "' != '' else 'imu_link'",
        ]
    )

    gps_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/gps_link' if '",
            agent_ns,
            "' != '' else 'gps_link'",
        ]
    )

    depth_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/depth_link' if '",
            agent_ns,
            "' != '' else 'depth_link'",
        ]
    )

    dvl_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/dvl_link' if '",
            agent_ns,
            "' != '' else 'dvl_link'",
        ]
    )

    beam0_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/beam0_link' if '",
            agent_ns,
            "' != '' else 'beam0_link'",
        ]
    )

    beam1_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/beam1_link' if '",
            agent_ns,
            "' != '' else 'beam1_link'",
        ]
    )

    beam2_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/beam2_link' if '",
            agent_ns,
            "' != '' else 'beam2_link'",
        ]
    )

    beam3_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/beam3_link' if '",
            agent_ns,
            "' != '' else 'beam3_link'",
        ]
    )

    com_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/com_link' if '",
            agent_ns,
            "' != '' else 'com_link'",
        ]
    )

    modem_link_frame = PythonExpression(
        [
            "'",
            agent_ns,
            "/modem_link' if '",
            agent_ns,
            "' != '' else 'modem_link'",
        ]
    )

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
            DeclareLaunchArgument(
                "loc_comparison",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            DeclareLaunchArgument(
                "lead_agent",
                default_value="",
                description="Namespace of the lead agent (optional)",
            ),
            Node(
                package="coug_fg",
                executable="factor_graph",
                name="factor_graph_node",
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "target_frame": dvl_link_frame,
                        "imu.parameter_frame": imu_link_frame,
                        "gps.parameter_frame": gps_link_frame,
                        "depth.parameter_frame": depth_link_frame,
                        "mag.parameter_frame": imu_link_frame,
                        "ahrs.parameter_frame": imu_link_frame,
                        "dvl.parameter_frame": dvl_link_frame,
                        "beams.beam0_parameter_frame": beam0_link_frame,
                        "beams.beam1_parameter_frame": beam1_link_frame,
                        "beams.beam2_parameter_frame": beam2_link_frame,
                        "beams.beam3_parameter_frame": beam3_link_frame,
                        "wrench.parameter_frame": com_link_frame,
                        "multiagent.parameter_frame": modem_link_frame,
                        "multiagent_base_frame": "base_link_nbr",
                        "multiagent.enable_multiagent": is_lead_agent,
                    },
                ],
            ),
            # iSAM2 (comparison)
            Node(
                package="coug_fg",
                executable="factor_graph",
                name="factor_graph_node_isam2",
                condition=IfCondition(loc_comparison),
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "target_frame": dvl_link_frame,
                        "imu.parameter_frame": imu_link_frame,
                        "gps.parameter_frame": gps_link_frame,
                        "depth.parameter_frame": depth_link_frame,
                        "mag.parameter_frame": imu_link_frame,
                        "ahrs.parameter_frame": imu_link_frame,
                        "dvl.parameter_frame": dvl_link_frame,
                        "beams.beam0_parameter_frame": beam0_link_frame,
                        "beams.beam1_parameter_frame": beam1_link_frame,
                        "beams.beam2_parameter_frame": beam2_link_frame,
                        "beams.beam3_parameter_frame": beam3_link_frame,
                        "wrench.parameter_frame": com_link_frame,
                        "multiagent.parameter_frame": modem_link_frame,
                        "multiagent_base_frame": "base_link_nbr",
                        "multiagent.enable_multiagent": is_lead_agent,
                        "global_odom_topic": "odometry/global_isam2",
                        "smoothed_path_topic": "smoothed_path_isam2",
                        "publish_global_tf": False,
                        "publish_smoothed_path": False,
                        "solver_type": "ISAM2",
                    },
                ],
            ),
            # Loosely-coupled DVL preintegration (comparison)
            Node(
                package="coug_fg",
                executable="factor_graph",
                name="factor_graph_node_lpi",
                condition=IfCondition(loc_comparison),
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "target_frame": dvl_link_frame,
                        "imu.parameter_frame": imu_link_frame,
                        "gps.parameter_frame": gps_link_frame,
                        "depth.parameter_frame": depth_link_frame,
                        "mag.parameter_frame": imu_link_frame,
                        "ahrs.parameter_frame": imu_link_frame,
                        "dvl.parameter_frame": dvl_link_frame,
                        "beams.beam0_parameter_frame": beam0_link_frame,
                        "beams.beam1_parameter_frame": beam1_link_frame,
                        "beams.beam2_parameter_frame": beam2_link_frame,
                        "beams.beam3_parameter_frame": beam3_link_frame,
                        "wrench.parameter_frame": com_link_frame,
                        "multiagent.parameter_frame": modem_link_frame,
                        "multiagent_base_frame": "base_link_nbr",
                        "multiagent.enable_multiagent": is_lead_agent,
                        "global_odom_topic": "odometry/global_lpi",
                        "smoothed_path_topic": "smoothed_path_lpi",
                        "publish_global_tf": False,
                        "comparison.enable_loose_dvl_preintegration": True,
                    },
                ],
            ),
            # Tightly-coupled DVL preintegration (comparison)
            Node(
                package="coug_fg",
                executable="factor_graph",
                name="factor_graph_node_tpi",
                condition=IfCondition(loc_comparison),
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "target_frame": dvl_link_frame,
                        "imu.parameter_frame": imu_link_frame,
                        "gps.parameter_frame": gps_link_frame,
                        "depth.parameter_frame": depth_link_frame,
                        "mag.parameter_frame": imu_link_frame,
                        "ahrs.parameter_frame": imu_link_frame,
                        "dvl.parameter_frame": dvl_link_frame,
                        "beams.beam0_parameter_frame": beam0_link_frame,
                        "beams.beam1_parameter_frame": beam1_link_frame,
                        "beams.beam2_parameter_frame": beam2_link_frame,
                        "beams.beam3_parameter_frame": beam3_link_frame,
                        "wrench.parameter_frame": com_link_frame,
                        "multiagent.parameter_frame": modem_link_frame,
                        "multiagent_base_frame": "base_link_nbr",
                        "multiagent.enable_multiagent": is_lead_agent,
                        "global_odom_topic": "odometry/global_tpi",
                        "smoothed_path_topic": "smoothed_path_tpi",
                        "publish_global_tf": False,
                        "comparison.enable_tight_dvl_preintegration": True,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="dvl_a50_twist_beams",
                name="dvl_a50_twist_beams_node",
                parameters=[
                    fleet_params,
                    agent_params,
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
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": "map",
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
                    fleet_params,
                    agent_params,
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
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "parameter_child_frame": gps_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="sbg_imu_mag",
                name="sbg_imu_mag_node",
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="seatrac_x150_imu_depth",
                name="seatrac_x150_imu_depth_node",
                parameters=[
                    fleet_params,
                    agent_params,
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
                    fleet_params,
                    agent_params,
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
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="coug_fg",
                executable="imu_ned_to_enu",
                name="imu_ned_to_enu_node",
                parameters=[
                    fleet_params,
                    agent_params,
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
                    fleet_params,
                    agent_params,
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
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
                condition=IfCondition(EqualsSubstitution(agent_ns, "bluerov2")),
            ),
            # --- Robot Localization Pipeline ---
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": odom_frame,
                        "base_link_frame": base_link_frame,
                        "world_frame": odom_frame,
                    },
                ],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                condition=IfCondition(loc_comparison),
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_link_frame": base_link_frame,
                        "world_frame": "map",
                    },
                ],
                remappings=[("odometry/filtered", "odometry/global_ekf")],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_map",
                condition=IfCondition(loc_comparison),
                parameters=[
                    fleet_params,
                    agent_params,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_link_frame": base_link_frame,
                        "world_frame": "map",
                    },
                ],
                remappings=[("odometry/filtered", "odometry/global_ukf")],
            ),
        ]
    )
