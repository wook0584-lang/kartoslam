import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  package_share_directory = get_package_share_directory('slam_karto')
  mapper_params = os.path.join(package_share_directory, 'config', 'mapper_params.yaml')

  slam_node = Node(
    package='slam_karto',
    executable='slam_karto',
    name='slam_karto',
    output='screen',
    parameters=[
      mapper_params,
      {
        'odom_frame': 'odom',
        'rangeThreshold': 30.0,
      },
    ],
  )

  return LaunchDescription([slam_node])
