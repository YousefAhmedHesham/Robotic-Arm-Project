"""
Launch file for the robot GUI control application
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ym_robotic_arm", 
            package_name="robot_moveit"
        )
        .robot_description(file_path="config/ym_robotic_arm.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("robot_moveit")
            + "/config/robot_motion_planning_python.yaml"
        )
        .to_moveit_configs()
    )

    # GUI Node
    gui_node = Node(
        name="robot_gui_control",
        package="robot_moveit",
        executable="robot_gui_control.py",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # Static TF Node
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # ROS2 Control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ym_robotic_arm"),
        "config",
        "joint_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, {"use_sim_time": True}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

    # Controller Spawners
    load_controllers = []
    for controller in [
        "arm_group_controller",
        "hand_group_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    return LaunchDescription(
        [
            robot_state_publisher,
            static_tf,
            ros2_control_node,
            gui_node,
        ]
        + load_controllers
    )