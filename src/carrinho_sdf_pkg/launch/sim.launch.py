from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('carrinho_sdf_pkg')
    world = os.path.join(pkg, 'worlds', 'meu_mundo.world')
    model = os.path.join(pkg, 'models', 'carrinho_sdf', 'model.sdf')

    ign = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '--verbose'],
        output='screen'
    )

    # Spawna o modelo SDF 
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'vehicle_blue', '-file', model, '-x', '0', '-y', '0', '-z', '0.30'],
        output='screen'
    )
    spawn = TimerAction(period=4.0, actions=[spawn_node])
    # Bridges
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        output='screen'
    )

    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
        output='screen'
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
        output='screen'
    )

    # TF do modelo 
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
        remappings=[('/model/vehicle_blue/tf', '/tf')],
        output='screen'
    )

    bridge_tf_static = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
        remappings=[('/model/vehicle_blue/tf_static', '/tf_static')],
        output='screen'
    )

    return LaunchDescription([
        ign,
        spawn,
        bridge_cmd_vel,
        bridge_odom,
        bridge_lidar,
        bridge_camera,
        bridge_imu,
        bridge_clock,
        bridge_tf,
        bridge_tf_static,
    ])
