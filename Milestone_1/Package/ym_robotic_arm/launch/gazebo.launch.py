from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ym_robotic_arm = get_package_share_directory('ym_robotic_arm')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Load URDF
    urdf_file = os.path.join(pkg_ym_robotic_arm, 'urdf', 'ym_robotic_arm.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ROS-GZ Bridge
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/model/ym_robotic_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.JointState'
    ],
    remappings=[('/model/ym_robotic_arm/joint_state', '/joint_states')],
    output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ym_robotic_arm', '-topic', '/robot_description'],
        output='screen',
    )

    # Controller configurations
    controllers_yaml = PathJoinSubstitution([
        pkg_ym_robotic_arm, 'config', 'joint_controllers.yaml'
    ])

    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        {"robot_description": robot_description},
        {"use_sim_time": LaunchConfiguration('use_sim_time')},
        PathJoinSubstitution([pkg_ym_robotic_arm, "config", "joint_controllers.yaml"])
    ],
    output="screen",
    )

    joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"],
    )
    
    hand_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["hand_group_controller", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription([
        use_sim_time,
        gazebo,
        spawn_robot,
        bridge,
        robot_state_publisher,
        control_node,
        joint_broadcaster,
        arm_controller,
        hand_controller,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', '/home/mahmouuudd/robotics_ws/src/ym_robotic_arm/meshes'),
    ])