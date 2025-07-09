arucoland

PX4 SITL + ROS 2 ArUco Detector & Precise Landing

    ✨ Before using this project, delete the pre-cloned .vscode folder to avoid configuration conflicts.

🔧 Requirements
1. Install OpenCV 4.10+

You must install OpenCV 4.10 or higher from source. Follow the official guide:
📎 https://docs.opencv.org/4.10.0/d7/d9f/tutorial_linux_install.html
2. Environment

    Ubuntu 22.04

    ROS 2 Humble

    Gazebo Harmonic

    PX4 SITL

    ros_gz (formerly ros_ign):
    Required for precise landing with PX4 SITL in Gazebo Harmonic.
    You must install it from source, matching your ROS and Gazebo version carefully:
    📎 https://github.com/gazebosim/ros_gz

🚀 How to Run
Terminal 1: PX4 SITL

cd ~/path/to/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco

Terminal 2: Micro XRCE DDS Agent

cd ~/path/to/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

Terminal 3: ros_gz_bridge

source ~/ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo

Terminal 4: Run ArUco Detector

cd ~/path/to/arucoland
source install/setup.bash

ros2 run aruco_detector aruco_detector

Terminal 5: Run Precise Land Node

cd ~/path/to/arucoland
source install/setup.bash

ros2 run precise_land precise_land

🖥️ (Optional) Visualize with RViz2

rviz2
