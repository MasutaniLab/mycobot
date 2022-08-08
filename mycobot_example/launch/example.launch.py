import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = load_file(
        'mycobot_280_description', 'urdf/mycobot_urdf.urdf')

    robot_description = {"robot_description":robot_description_config}

    robot_description_semantic_config = load_file(
        'mycobot_280_description', 'urdf/mycobot_280.srdf.xacro')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('mycobot_280_description', 'config/kinematics.yaml')

    example_node = Node(name='mycobot_example_node',
                        package='mycobot_example',
                        executable='mycobot_example',
                        output='screen',
                        parameters=[robot_description,
                                    robot_description_semantic,
                                    kinematics_yaml])

    return LaunchDescription([example_node])