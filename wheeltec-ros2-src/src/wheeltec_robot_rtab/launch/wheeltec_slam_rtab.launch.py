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
    use_astrapro = LaunchConfiguration('use_astrapro', default='false')

    astra_dir = get_package_share_directory('ros2_astra_camera')
    wheeltec_slam_dir = get_package_share_directory('wheeltec_slam_toolbox')
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_slam_dir, 'launch', 'online_sync.launch.py')
        ),
    )
    rplidar_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    depth_img = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            astra_dir, 'launch', 'astra_pro_launch.py')),
        condition=IfCondition(use_astrapro),
    )
    parameters = {
        'queue_size': 20,
        'frame_id': 'camera_link',
        'use_sim_time': use_sim_time,
        'publish_tf': True,

        # check this link for choice of optimizers
        # https://answers.ros.org/question/343139/rtabmap-g2o-gtsam/
        'Optimizer/Strategy': '0',  # defaults to 1

        'Optimizer/Slam2D': 'true',
        'Optimizer/Robust': 'true',
        'wait_for_transform_duration': 0.001,
        'wait_for_transform': 0.001,
        # this is important to avoid map bluring due to odom noise
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'Reg/Strategy': '2',
        'Reg/Force3DoF': 'true',
        'Vis/InlierDistance': '0.5',
        'Icp/MaxRotation': '1.57',
        # this removes existing obstacle in 2d map, but not working in 3d
        'Grid/RayTracing': 'true',
        'map_filter_radius': 0.5,
        'map_always_update': True,

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

        # wheeltec_robot,
        # rplidar_ros,
        # depth_img,
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument('use_astrapro', default_value='false'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),

        # Nodes to launch
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters,
                        {'Mem/IncrementalMemory': 'False', 'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),
    ])
