import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    robot_control_node = Node(
        package='autorace_core_ArchieRobotics',
        executable='ros_competition_node',
        name='robot_competition_node',
        output='screen'
    )

    task_1_node = Node(
        package='autorace_core_ArchieRobotics',
        executable='task_1_node',
        name='task_1_node',
        output='screen'
    )

    task_2_node = Node(
        package='autorace_core_ArchieRobotics',
        executable='task_2_node',
        name='task_2_node',
        output='screen'
    )

    task_3_node = Node(
        package='autorace_core_ArchieRobotics',
        executable='task_3_node',
        name='task_3_node',
        output='screen'
    )


    yolo_node = Node(
        package='autorace_core_ArchieRobotics',
        executable='yolo_node',
        name='yolo_node',
        output='screen'
    )


    drive_node = Node(
        package='autorace_core_ArchieRobotics',
        executable='drive_node',
        name='drive_node',
        output='screen'
    )

    return LaunchDescription([
        robot_control_node,
        task_1_node,
        task_2_node,
        task_3_node,
        yolo_node,
        drive_node,
    ])