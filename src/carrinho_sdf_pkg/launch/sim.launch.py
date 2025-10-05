from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('carrinho_sdf_pkg')
    world = os.path.join(pkg, 'worlds', 'meu_mundo.world')
    model = os.path.join(pkg, 'models', 'carrinho_sdf', 'model.sdf')

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'carrinho_sdf', '-file', model, '-x', '0', '-y', '0', '-z', '0.30'],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn])
