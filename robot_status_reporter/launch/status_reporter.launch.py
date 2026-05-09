from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    with_echo_arg = DeclareLaunchArgument(
        'with_echo',
        default_value='false',
        description='是否同时启动 `ros2 topic echo /robot_integrated_status` 来显示聚合结果',
    )

    status_node = Node(
        package='robot_status_reporter',
        executable='status_aggregator',
        name='status_aggregator',
        output='screen',
    )

    echo_status = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/robot_integrated_status'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_echo')),
    )

    tip = LogInfo(
        msg='status_aggregator 已启动；若上游话题(/battery_state, /medicine_inventory, TF map->base_link)可用，将持续输出 /robot_integrated_status。'
    )

    return LaunchDescription([
        with_echo_arg,
        tip,
        status_node,
        echo_status,
    ])
