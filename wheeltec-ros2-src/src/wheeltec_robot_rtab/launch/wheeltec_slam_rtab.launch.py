import os
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    database_path = LaunchConfiguration('database_path', default='~/.ros/rtabmap.db')
    parameters = {
        'queue_size': 20,
        'frame_id': 'camera_link',
        'use_sim_time': use_sim_time,
        'database_path': database_path,

        # check this link for choice of optimizers
        # https://answers.ros.org/question/343139/rtabmap-g2o-gtsam/
        'Optimizer/Strategy': '0',  # defaults to 1

        'Optimizer/Slam2D': 'true',
        'Optimizer/Robust': 'true',
        'wait_for_transform_duration': 0.010,
        'wait_for_transform': 0.010,
        # this is important to avoid map bluring due to odom noise
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',

        # do not add node if moving fast
        'RGBD/AngularSpeedUpdate': '0.01',
        'RGBD/LinearSpeedUpdate': '0.30',
        'odom_sensor_sync': True,

        'Reg/Strategy': '2',
        'Reg/Force3DoF': 'true',
        # 'Vis/InlierDistance': '1.0',
        # 'Icp/MaxRotation': '1.57',
        # this removes existing obstacle in 2d map, but not working in 3d
        'Grid/RayTracing': 'true',
        'map_filter_radius': 0.2,
        # 'map_always_update': True,

        'approx_sync': True,

        'subscribe_scan': True,
        'subscribe_depth': True,
        'Grid/Sensor': '2',
        'Grid/CellSize': '0.01'}

    remappings = [
        ('odom', 'odom_combined'),
        ('scan', 'scan'),
        ('rgb/image', 'camera/color/image_raw'),
        ('rgb/camera_info', 'camera/color/camera_info'),
        ('depth/image', 'camera/depth/image')]

    namespace = f'/robots/{os.environ["ROBOT_NAME"]}'

    localization = LaunchConfiguration('localization')

    return LaunchDescription([
        PushRosNamespace(namespace=namespace),

        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),

        SetRemap(src='map', dst='/shared/map'),
        SetRemap(src='cloud_map', dst='/shared/cloud_map'),

        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Database path.'),

        # Nodes to launch
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters, {'publish_tf': True}],
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters,
                {'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true', 'publish_tf': False, 'subscribe_depth' : False, 'subscribe_rgb' : False, 'subscribe_rgbd' : False}],
            remappings=remappings,
            arguments=[]),
    ])
