from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import math

def generate_launch_description():
    h = math.pi/2
    p = math.pi
    return LaunchDescription([
        # LEFT
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
        # ),

        # COMBINED CLOUDS
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'combined_cloud'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            # arguments=['0', '1', '1.5', f'{math.pi/2}', '0', f'-{math.pi/2}', 'base_link', 'left_sensor'],
            arguments=['0', '-0.2', '1.5', f'{h}', f'{p}', f'{h}', 'base_link', 'left_sensor'],

        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            # arguments=['0', '-1', '1.5', f'-{math.pi/2}', '0', f'{math.pi/2}', 'base_link', 'right_sensor'],
            arguments=['0', '0.2', '1.5', f'{-h}', f'{p}', f'{-h}', 'base_link', 'right_sensor'],
        ),

        # LEFT
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'left_sensor', 'left_cloud'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring1_235" args="0 0 0 0 0.0785 0 left_sensor scanner_2_1 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0.0785', '0', '0', '0', '0', 'left_sensor', 'left_scanner_1'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring2_235" args="0 0 0 0 0.0262 0 left_sensor scanner_2_2 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0.0262', '0', '0', '0', '0', 'left_sensor', 'left_scanner_2'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring3_235" args="0 0 0 0 -0.0785 0 left_sensor scanner_2_3 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '-0.0785', '0', '0', '0', '0', 'left_sensor', 'left_scanner_3'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring4_235" args="0 0 0 0 -0.0262 0 left_sensor scanner_2_4 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '-0.0262', '0', '0', '0', '0', 'left_sensor', 'left_scanner_4'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring1_236" args="0 0 0 0 0.0785 0 right_sensor scanner_3_1 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0.0785', '0', '0', '0', '0', 'right_sensor', 'right_scanner_1'],
        ),

        # RIGHT
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'right_sensor', 'right_cloud'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring2_236" args="0 0 0 0 0.0262 0 right_sensor scanner_3_2 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0.0262', '0', '0', '0', '0', 'right_sensor', 'right_scanner_2'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring3_236" args="0 0 0 0 -0.0785 0 right_sensor scanner_3_3 10" />
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '-0.0785', '0', '0', '0', '0', 'right_sensor', 'right_scanner_3'],
        ),
        Node (
        # <node pkg="tf" type="static_transform_publisher" name="ring4_236" args="0 0 0 0 -0.0262 0 right_sensor scanner_3_4 10" /> 
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '-0.0262', '0', '0', '0', '0', 'right_sensor', 'right_scanner_4'],
        ),

        Node(
            package='cloud_transform',
            executable='cloud_transform',
            name='cloud_transform',
            output='screen',
        ),
    ])
