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
    slam_params = os.path.join(pkg_share, 'config/slam_params.yaml')
    map_yaml = os.path.join(pkg_share, 'maps/map.yaml')

    default_model_path = os.path.join(dscrptn_share, 'src/description/robot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/world_only.sdf')

    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time' : True}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]    

    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': True}]
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
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        # robot_localization_node,
        rviz_node,
        # GroupAction(
        # actions=[
        #     launch_ros.actions.SetParameter("use_sim_time", LaunchConfiguration('use_sim_time')),
        #     launch_ros.actions.Node(
        #         package='nav2_map_server',
        #         executable='map_saver_server',
        #         output='screen'),
        #     launch_ros.actions.Node(
        #         package='nav2_lifecycle_manager',
        #         executable='lifecycle_manager',
        #         name='lifecycle_manager_slam',
        #         output='screen'
        #         )
        # ]),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch/online_sync_launch.py')),
        #     launch_arguments={'use_sim_time': 'True','slam_params_file': slam_params}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch/bringup_launch.py')),
            launch_arguments={'params_file': params_file, "use_sim_time": 'True', 'map': map_yaml}.items())

    ])