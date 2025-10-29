from setuptools import setup, find_packages

package_name = 'my_multi_plot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/plot_listener.launch.py']),
    ],
    install_requires=['setuptools', "influxdb-client"],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Multi-board temperature/humidity plot subscriber with auto-discovery.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'plot_listener = my_multi_plot.plot_listener:main',
            'sht40_logger = my_multi_plot.sht40_arduino_ros_logger:main',
            'sht40_cloud_sender = my_multi_plot.sht40_ROS_cloud_sender:main',
        ],
    },
)
