"""Launch Gazebo Classic + textured room + ov_rover + teleop/auto drive."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ov_rover_sim')
    world = os.path.join(pkg, 'worlds', 'small_room.world')
    urdf = os.path.join(pkg, 'urdf', 'rover.urdf')
    model_sdf = os.path.join(pkg, 'models', 'ov_rover', 'model.sdf')

    with open(urdf) as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world,
                          'verbose': 'false',
                          'gui': LaunchConfiguration('gui')}.items())

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        # z=0.01: wheels touch gently (0.1 drops/bounces the rover into a spin)
        arguments=['-file', model_sdf, '-entity', 'ov_rover', '-x', '0', '-y', '0', '-z', '0.01'],
        output='screen')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}], output='screen')

    auto = Node(package='ov_rover_sim', executable='auto_loop.py',
                parameters=[{'enabled': LaunchConfiguration('auto')}],
                condition=IfCondition(LaunchConfiguration('auto')),
                output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='true', description='drive square loop automatically'),
        DeclareLaunchArgument('rviz', default_value='false', description='open rviz'),
        DeclareLaunchArgument('gui', default_value='true', description='gazebo GUI (false=headless)'),
        gazebo, spawn, rsp, auto,
        # manual drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'rover.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
