arucoland: PX4 SITL Precision Landing with ROS 2 and ArUco Detection

This repository contains the necessary components to perform precision landing for a PX4 drone (simulated in Gazebo) using ROS 2, ArUco markers, and a downward-facing monocular camera.
⚠️ Important Pre-check

Before you begin, please ensure you delete any local configuration files like .vscode that might have been cloned if you forked this repository, as they might conflict with your environment.
Prerequisites & Setup
1. OpenCV Installation

You need OpenCV version 4.10 or newer. It is highly recommended to install it from source. Follow the official guide:

    OpenCV Installation Guide (Linux)

2. ROS-Gazebo Bridge (ros_gz) Installation

This setup specifically targets Ubuntu 22.04 (Humble Hawksbill) and Gazebo Harmonic. To bridge topics between Gazebo and ROS 2 for precision landing with PX4 SITL, you'll likely need to install ros_gz from source.

Crucially, ensure you install the version compatible with your ROS 2 and Gazebo setup to avoid compilation or runtime errors.

    ros_gz GitHub Repository

3. PX4-Autopilot & Micro-XRCE-DDS-Agent

Ensure you have your PX4-Autopilot repository set up and the Micro-XRCE-DDS-Agent cloned and ready.
Execution Steps

Open five separate terminal windows and execute the commands in the order specified below. Replace ~/.../ with your actual project paths.
Terminal 1: Start PX4 SITL (Gazebo Simulation)

Navigate to your PX4-Autopilot directory and launch the Gazebo simulation with the x500_mono_cam_down_aruco model.
Generated bash

      
cd ~/path/to/your/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
Terminal 2: Start Micro-XRCE-DDS Agent

Navigate to your Micro-XRCE-DDS-Agent directory and start the DDS agent. This acts as the bridge between PX4 (DDS) and ROS 2 (DDS-XRCE).
Generated bash

      
cd ~/path/to/your/Micro-XRCE-DDS-Agent
./MicroXRCEAgent udp4 -p 8888

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
Terminal 3: Start ROS-Gazebo Bridge

First, source your ROS 2 workspace, then launch the parameter_bridge to transfer camera image and info topics from Gazebo to ROS 2.
Generated bash

      
source ~/ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
Terminal 4: Start ArUco Detector

Navigate to your arucoland workspace, source it, and then run the ArUco detector node. This node will process the camera image to detect ArUco markers.
Generated bash

      
cd ~/path/to/your/arucoland
source install/setup.bash
ros2 run aruco_detector aruco_detector

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
Terminal 5: Start Precision Landing Node

In the final terminal, navigate to your arucoland workspace again, source it, and launch the precision landing node. This node will use the ArUco detection results to compute and send precise landing commands to the PX4 drone.
Generated bash

      
cd ~/path/to/your/arucoland
source install/setup.bash
ros2 run precise_land precise_land

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
(Optional) Visualization with Rviz2

You can visualize the camera feed, ArUco detections, and other ROS 2 topics using Rviz2. Open a new terminal (after sourcing your workspace) and run:
Generated bash

      
source ~/ws/install/setup.bash # Make sure to source if it's a new terminal
rviz2

    
