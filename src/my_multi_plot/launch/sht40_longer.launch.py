from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_multi_plot',
            executable='sht40_logger',      # <- matches setup.py console_scripts name
            name='sht40_logger',
            output='screen',
            # You can pass your CLI flags here if you like:
            # arguments=['--results-dir', 'Results', '--pair-tolerance-s', '0.2', '--stale-timeout-s', '2.0']
        ),
    ])
