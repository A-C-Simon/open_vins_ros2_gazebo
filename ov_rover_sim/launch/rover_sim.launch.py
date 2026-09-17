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

    # Everything simulated runs on sim time so behavior is identical at any
    # real-time factor (steady headless slow-motion would otherwise shrink
    # every auto phase and stamp TFs with the wrong clock).
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc},
                           {'use_sim_time': True}],
               output='screen')

    auto = Node(package='ov_rover_sim', executable='auto_loop.py',
                parameters=[{'enabled': LaunchConfiguration('auto')},
                            {'mode': LaunchConfiguration('mode')},
                            {'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('auto')),
                output='screen')

    # One-time global->odom alignment: wheel odometry lives in `odom`,
    # OpenVINS in its own `global` frame, and VIO global yaw is
    # unobservable, so the two differ by a fixed yaw. The node measures
    # it from the first meters both travel and latches the corrected
    # static transform (identity fallback if VINS never appears, e.g.
    # --no-vins, so RViz still has a full TF tree). After that, any
    # remaining separation between the paths is real estimator drift.
    # Estimation itself never uses TF.
    align = Node(package='ov_rover_sim', executable='align_frames.py',
                 parameters=[{'use_sim_time': True}],
                 output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='true', description='drive automatically'),
        DeclareLaunchArgument('mode', default_value='circle', description='auto drive mode: circle or square'),
        DeclareLaunchArgument('rviz', default_value='false', description='open rviz'),
        DeclareLaunchArgument('gui', default_value='true', description='gazebo GUI (false=headless)'),
        gazebo, spawn, rsp, auto, align,
        # manual drive: ros2 run ov_rover_sim key_teleop.py
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'rover.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
