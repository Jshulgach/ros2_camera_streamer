# ros2_camera_streamer
A ROS2 package for streaming from USB webcams

## Setup Instructions

### Prerequisites
Ensure you have the following dependencies installed on your system:
- **ROS 2 (e.g., Humble, Iron, or Jazzy)**: Follow the official ROS 2 installation guide for your platform.
- **Vision OpenCV (ROS package)**: Install using:
  ```bash
  sudo apt install ros-<distro>-vision-opencv
  ```

- Get the boost python development library
  ```bash
  sudo apt install libboost-python-dev
  ```

## Building the package
Clone the repository into your colcon workspace `src` folder
```bash
git clone https://github.com/Jshulgach/ros2_camera_streamer.git
```
Build package
```bash
cd ..
colcon build --packages-select ros2_camera_streamer
source install/setup.bash
```

## Usage
The streamer can be run with the terminal command 
```bash
ros2 run ros_camera_streamer usbcam_stream
```




