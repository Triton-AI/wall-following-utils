import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from glob import glob


def generate_launch_description():
    sensor_pkg = 'basestation_launch'
    some_package = 'gap_follow'
    some_node = 'ReactiveFollowGap'    
    some_config = 'gap_follower_config.yaml'

   # ld = LaunchDescription()
   # config = os.path.join(
   #     get_package_share_directory(some_package), # change to "some_package" for local testing usage
   #     'config',
   #     some_config)
        
    pid_node=Node(
        executable=some_node,
        package=some_package,
        output='screen',
        parameters=[config]
    )

    ## tf2 - lidar_link to laser
    # node_tf2_fp2laser = Node(
    #     name='tf2_ros_fp_laser',
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'lidar_link', 'laser'],   
    # )

    ld.add_action(pid_node)
    # ld.add_action(node_tf2_fp2laser)
    return ld
