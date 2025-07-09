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
