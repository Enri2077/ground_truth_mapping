from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use'),
        launch_ros.actions.Node(
            package='slam_toolbox',
            node_executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
    ])
