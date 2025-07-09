# arucoland  
**PX4 SITL + ROS 2 ArUco Detector & Precise Landing**

---

## 📌 Notice

> ❗ **Before using this project**, delete the pre-cloned `.vscode` folder from the repository to avoid configuration conflicts.

---

## 📦 Requirements

### ✅ Environment
- Ubuntu **22.04**
- ROS 2 **Humble**
- Gazebo **Harmonic**
- PX4 **SITL**
- `ros_gz` (formerly `ros_ign`) — must be installed **from source** and carefully matched to your ROS & Gazebo versions  
  🔗 https://github.com/gazebosim/ros_gz

### ✅ OpenCV 4.10+
You **must install OpenCV 4.10 or newer** from source. Follow the guide below:  
🔗 https://docs.opencv.org/4.10.0/d7/d9f/tutorial_linux_install.html

### ✅ Before start!!!
```bash
cd ~/path/to/PX4-Autopilot/Tools/simulation/gz/worlds
```

Replace or create the file aruco.sdf with the following content
```aruco.sdf
﻿<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="aruco">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <include>
      <uri>model://arucotag</uri>
      <pose>10 0 0.05 0 0 0</pose>
    </include>
  </world>
</sdf>
```

---

## 🚀 How to Run

### 1️⃣ Terminal 1 — Launch PX4 SITL
```bash
cd ~/path/to/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco
```

### 2️⃣ Terminal 2 — Start Micro XRCE DDS Agent
```bash
cd ~/path/to/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```

### 3️⃣ Terminal 3 — Start ros_gz_bridge
```bash
source ~/ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### 4️⃣ Terminal 4 — Run ArUco Detector Node
```bash
cd ~/path/to/arucoland
source install/setup.bash

ros2 run aruco_detector aruco_detector
```

### 5️⃣ Terminal 5 — Run Precise Landing Node
```bash
cd ~/path/to/arucoland
source install/setup.bash

ros2 run precise_land precise_land
```

### 🖥️ Optional: Visualize in RViz2
```bash
rviz2
```

### 📁 Directory Structure (optional)
```bash
arucoland/
├── aruco_detector/         # ArUco marker detector ROS 2 node
├── precise_land/           # Precision landing controller node
├── install/                # Colcon install folder (after build)
├── src/                    # Source code
├── .vscode/ (delete this!)
└── README.md
```

 ### Tips

    Be careful with version mismatches between ROS, Gazebo, and ros_gz.

    If image topics don’t show up, check if ros_gz_bridge is relaying the camera topic correctly.

    Test your OpenCV install with pkg-config --modversion opencv4.
