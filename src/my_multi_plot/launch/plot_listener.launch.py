from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Pass CLI args after the node name using 'arguments'
    return LaunchDescription([
        Node(
            package='my_multi_plot',
            executable='plot_listener',
            name='plot_listener',
            output='screen',
            arguments=['--window-seconds', '120']  # tweak as you like
        ),
    ])
