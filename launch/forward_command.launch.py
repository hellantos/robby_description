#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import UnlessCondition


def generate_launch_description():
    arg_use_real_hardware = DeclareLaunchArgument(
            "use_real_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
    )

    arg_can_interface_name = DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="Start robot with mock hardware mirroring command to its states.",
    )

    can_interface_name = LaunchConfiguration("can_interface_name")
    use_real_hardware = LaunchConfiguration("use_real_hardware")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robby_description"),
                    "urdf",
                    "robby.system.xacro",
                ]
            ),
            " ",
            "use_mock_hardware:=", "false", " ",
            "can_interface_name:=", can_interface_name
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("robby_description"), "config/robby", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robby_forward_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log"
    )

    slave_config = PathJoinSubstitution(
        [FindPackageShare("robby_description"), "config/robby", "TMCM-1270.eds"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "fake_left_drive",
            "slave_config": slave_config,
        }.items(),
        condition=UnlessCondition(use_real_hardware)
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "fake_right_drive",
            "slave_config": slave_config,
        }.items(),
        condition=UnlessCondition(use_real_hardware)
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_master_launch = TimerAction(
        period=1.0,
        actions=[
            control_node, 
            joint_state_broadcaster_spawner, 
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner]
    )

    delay_slave_launch = TimerAction(
        period=2.0,
        actions=[
            slave_node_1,slave_node_2]
    )

    nodes_to_start = [
        arg_can_interface_name,
        arg_use_real_hardware,
        robot_state_publisher_node,
        delay_master_launch,
        delay_slave_launch
    ]

    return LaunchDescription(nodes_to_start)
