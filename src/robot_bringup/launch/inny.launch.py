import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_bringup').find('robot_bringup')
    dscrptn_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    nav2_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    slam_share = launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    params_file = os.path.join(pkg_share, 'config/nav2_params.yaml')


    default_model_path = os.path.join(dscrptn_share, 'src/description/tmp.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')

    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        remappings=[
            ("/robot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )


    


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        # joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        # spawn_entity,
        # robot_localization_node,
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch/online_async_launch.py'))),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch/navigation_launch.py')),
        #     launch_arguments={'params_file': params_file, "use_sim_time": 'True'}.items())

    ])