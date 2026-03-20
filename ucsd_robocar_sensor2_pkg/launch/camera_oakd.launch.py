import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    some_node = 'oakd_shared_node'
    calibration_file = os.path.join(
        get_package_share_directory('ucsd_robocar_lane_detection2_pkg'),
        'config',
        'ros_racer_calibration.yaml')

    original_topic_name = 'camera/color/image_raw_0'
    new_topic_name = LaunchConfiguration('topic_name', default=original_topic_name)

    ld = LaunchDescription()

    sensor_node=Node(
        package=sensor_pkg,
        executable=some_node,
        output='screen',
        parameters=[
            calibration_file,
            {'camera_topic': new_topic_name},
        ]
        )
    ld.add_action(sensor_node)
    return ld
