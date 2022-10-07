import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    # Get the launch directory
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
    
    my_nav_dir = get_package_share_directory('wheeltec_nav2')
    my_launch_dir = os.path.join(my_nav_dir, 'launch')
    my_map_dir = os.path.join(my_nav_dir, 'map')
    my_map_file = 'WHEELTEC.yaml'

    #Modify the model parameter file, the options are：
    #param_mini_akm.yaml、param_mini_4wd.yaml、param_mini_diff.yaml、
    #param_mini_mec.yaml、param_mini_omni.yaml、param_mini_tank.yaml、
    #param_senior_akm.yaml、param_senior_diff.yaml、param_senior_mec_bs.yaml
    #param_senior_mec_dl.yaml、param_top_4wd_bs.yaml、param_top_4wd_dl.yaml
    #param_top_akm_dl.yaml、param_four_wheel_diff_dl.yaml、param_four_wheel_diff_bs.yaml
    #If you want to use the DWB algorithm, you can change the input parameters to wheeltec-dwb.yaml
    my_param_dir = os.path.join(my_nav_dir, 'param','wheeltec_param')
    my_param_file = 'param_mini_mec.yaml'


    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam', default='false')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(my_map_dir, my_map_file),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether run a SLAM')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(my_param_dir, my_param_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='Launch Rviz?')
        
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    lidar_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_launch_dir, 'wheeltec_lidar.launch.py')),
    )
    namespace = f'/robots/{os.environ["ROBOT_NAME"]}'
    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        # FIXME: doesn't work if use relative path 'map' as src
        SetRemap(src='/robots/robot3/map', dst='/shared/map'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'slam_launch.py')),
            condition=IfCondition(use_slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        # IncludeLaunchDescription(
        #     # Run Localization only when we don't use SLAM
        #     PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'localization_launch.py')),
        #     condition=UnlessCondition(use_slam),
        #     launch_arguments={'namespace': namespace,
        #                       'map': map_yaml_file,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': params_file,
        #                       'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'rviz_view_launch.py')),
        #     launch_arguments={'use_sim_time': use_sim_time,
        #                       'open_rviz': open_rviz,
        #                       'map_subscribe_transient_local': 'true'}.items()),

    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    # ld.add_action(wheeltec_robot)
    # ld.add_action(lidar_ros)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_open_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
