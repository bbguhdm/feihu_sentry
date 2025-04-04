from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    parameters={
          'frame_id':'base_link',
          'map_frame_id':'map',
          'odom_frame_id':'odom_body',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_depth':True,
          #'publish_tf':False,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'RGBD/StartAtOrigin':'True',
          'Grid/RangeMin':'0.25', # ignore laser scan points on the robot itself
          'Grid/RangeMax':'10',
          'Grid/MaxObstacleHeight':'0.36',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/color/camera_info'),
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
          ('map','/map'),
          ('odom', '/odom')
        ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, 
                              description='Configuration path of rviz2.'),
        
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ (optional).'),


        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.01, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),
        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     parameters=[{
        #   'frame_id':'livox_frame',
        #   'subscribe_depth':True,
        #   'subscribe_odom_info':True,
        #   'approx_sync':False}],
        #     remappings=[
        #   ('rgb/image', '/camera/camera/color/image_raw'),
        #   ('rgb/camera_info', '/camera/camera/color/camera_info'),
        #   ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')]),
        #SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', 
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

    ])