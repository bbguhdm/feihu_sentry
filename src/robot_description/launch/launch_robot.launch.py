from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model = DeclareLaunchArgument(
        'model',
        default_value=[get_package_share_directory('fishbot_description'), '/urdf/auto_car.xacro'],
        description='Path to the model file to load')

    robot_description = Node(
        package='xacro',
        executable='xacro',
        arguments=['$(var model)'],
        parameters=[{'robot_description': '/dev/stdin'}]
    )

    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('robot_base'), '/launch/robot_base.launch.py'])
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ldlidar_sl_ros2'), '/launch/ld14.launch.py'])
    )

    serial_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('serial_imu'), '/launch/imu_0x91_msg.launch.py'])
    )

    joint_state_update_node = Node(
        package='auto_control',
        executable='joint_state_update_node',
        name='joint_state_update_node',
        output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace='/auto_car',
        parameters=[{'publish_frequency': 50.0}]
    )

    return LaunchDescription([
        model,
        robot_description,
        robot_base_launch,
        rplidar_launch,
        serial_imu_launch,
        joint_state_update_node,
        robot_state_publisher
    ])

