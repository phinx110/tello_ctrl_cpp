import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch a single drone.


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello.urdf')
    return LaunchDescription([
        # Rviz
        ExecuteProcess(cmd=['rviz2', '-d', 'src/tello_ctrl_cpp/launch/one.rviz'], output='screen'),

        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf]),

        # Driver
        Node(package='tello_driver', node_executable='tello_driver', output='screen',
             node_name='tello_driver', node_namespace='solo'),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen'),

        # Visual localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace='solo'),

        # Kalman filter
        Node(package='odom_filter', node_executable='filter_node', output='screen',
             node_name='filter_node', node_namespace='solo')
    ])
