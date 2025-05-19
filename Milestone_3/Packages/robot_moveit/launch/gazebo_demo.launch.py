# import the packages
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lab_gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ym_robotic_arm'), 'launch'), '/gazebo.launch.py']))
    
    moveit_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('robot_moveit'), 'launch'), '/demo.launch.py']))
    
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        lab_gazebo,
        moveit_node,
    ])