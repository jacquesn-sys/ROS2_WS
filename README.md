# ROS2_WS

Repo containing ROS2 workspace for handling PI-Arduino-SHT40 communication.

## Overview

This repository provides a ROS2 workspace designed to facilitate communication between a Raspberry Pi, Arduino, and an SHT40 sensor. The primary programming language is Python, but there are also components in PowerShell, Shell, and Dockerfile for environment setup and automation.

## Features

- ROS2 workspace structure
- Scripts and nodes for managing PI-Arduino-SHT40 data exchange
- Multi-language support for automation and setup

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/jacquesn-sys/ROS2_WS.git
   cd ROS2_WS
   ```

2. **Install dependencies:**
   Ensure you have ROS2 installed on your system. Refer to the official [ROS2 installation guide](https://docs.ros.org/) for your platform.

3. **Setup the workspace:**
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   colcon build
   source install/setup.bash
   ```

## Usage

- Launch the workspace as needed for your application.
- Refer to individual package and script documentation for specific usage details.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

No license specified.

## Contact

For questions or suggestions, reach out via [GitHub Issues](https://github.com/jacquesn-sys/ROS2_WS/issues).
