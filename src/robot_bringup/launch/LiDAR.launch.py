#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('robot_bringup')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'bluesea_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'config', 'LDS-50C-2.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

     #对多版本的兼容性处理 首字母e以及之前为18.04或更早,之后为20.04或更晚
    ROS_DISTRO=''
    ROS_DISTRO = os.getenv('ROS_DISTRO')
    print("Current ROS2 Version: ",ROS_DISTRO)
    if ROS_DISTRO[0] <= 'e':
        try:
            driver_node = LifecycleNode( node_name='bluesea_node', node_namespace='/', package='bluesea2', node_executable='bluesea_node', output='screen', parameters=[parameter_file])
        except:
            pass
    else :
        try:
            driver_node = LifecycleNode( name='bluesea_node', namespace='/', package='bluesea2', executable='bluesea_node', output='screen', emulate_tty=True, parameters=[parameter_file])
        except:
            pass


    return LaunchDescription([
        params_declare,
        driver_node
    ])