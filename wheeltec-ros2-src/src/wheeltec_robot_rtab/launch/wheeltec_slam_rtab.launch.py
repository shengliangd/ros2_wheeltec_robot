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
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_astrapro = LaunchConfiguration('use_astrapro', default='false')
    
    astra_dir = get_package_share_directory('ros2_astra_camera')
    wheeltec_slam_dir = get_package_share_directory('wheeltec_slam_toolbox')
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_slam = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(wheeltec_slam_dir, 'launch', 'online_sync.launch.py')
          ),
     )
    rplidar_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    depth_img = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(astra_dir,'launch', 'astra_pro_launch.py')),
            condition=IfCondition(use_astrapro),
            )
    parameters=[{
          'queue_size':20,
          'frame_id':'camera_link',
          'use_sim_time':use_sim_time,
          'subscribe_scan':True,
          'subscribe_depth':True}]

    remappings=[
          ('odom', 'odom_combined'),
          ('scan', 'scan'),
          ('rgb/image', 'camera/color/image_raw'), 
          ('rgb/camera_info', 'camera/color/camera_info'),
          ('depth/image', 'camera/depth/image')]
    namespace = f'/robots/{os.environ["ROBOT_NAME"]}'
    return LaunchDescription([
        GroupAction([
                PushRosNamespace(namespace=namespace),
                SetRemap(src='/tf',dst='tf'),
                SetRemap(src='/tf_static',dst='tf_static'),

                wheeltec_robot,rplidar_ros,depth_img,
                # Set env var to print messages to stdout immediately
                SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

                # Launch arguments
                DeclareLaunchArgument('use_astrapro',default_value='false'),
                DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'),

                # Nodes to launch
                Node(
                package='rtabmap_ros', executable='rtabmap', output='screen',
                parameters=parameters,
                remappings=remappings,
                arguments=['-d']),
        ])
        ])
