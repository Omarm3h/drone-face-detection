# 🚁 Autonomous Drone Face Detection System

An autonomous quadcopter system that performs face detection using YOLO (You Only Look Once) in a simulated urban environment. The drone executes a zigzag search pattern over a boulevard area, detects and tracks people using computer vision, and logs mission data.



## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## 🎯 Overview

This project demonstrates an autonomous drone system capable of:
- **Autonomous flight** using PX4 autopilot
- **Computer vision-based detection** using YOLOv8
- **Real-time face/person tracking** with ROS2 integration
- **Mission planning** with zigzag search patterns
- **Gazebo simulation** with realistic urban environment


---

## ✨ Features

- ✅ **Autonomous Navigation**: Pre-programmed flight paths with obstacle awareness
- ✅ **Real-time Detection**: YOLOv8-based person detection at 30 FPS
- ✅ **ROS2 Integration**: Modular architecture using ROS2 Humble
- ✅ **Gazebo Simulation**: Realistic physics and sensor simulation
- ✅ **Mission Logging**: Automatic path plotting and data recording
- ✅ **Configurable Parameters**: Easy adjustment of detection thresholds, flight patterns
- ✅ **GPU Acceleration**: Optional CUDA support for faster inference

---

## 🏗️ System Architecture

```
┌─────────────────┐
│  Gazebo World   │
│  (Boulevard)    │
│  - Buildings    │
│  - People       │
│  - Runway       │
└────────┬────────┘
         │
    ┌────▼────┐
    │  X500   │
    │  Drone  │
    │ +Camera │
    └────┬────┘
         │ /face_camera
    ┌────▼──────────┐
    │ ROS-GZ Bridge │
    └────┬──────────┘
         │ /camera/image_raw
    ┌────▼────────────────┐
    │ YOLO Face Detector  │
    │   (YOLOv8)          │
    └────┬────────────────┘
         │
         ├──► /face_detection/detected
         ├──► /face_detection/position
         └──► /face_detection/visualization
              │
         ┌────▼────────────┐
         │ Mission Script  │
         │ (MAVSDK)        │
         └─────────────────┘
              │
         ┌────▼────────┐
         │ PX4 SITL    │
         └─────────────┘
```

---

## 📦 Prerequisites

### Software Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Gazebo**: Harmonic
- **PX4 Autopilot**: Latest stable
- **Python**: 3.10+
- **CUDA**: 11.8+ (optional, for GPU acceleration)

### Hardware Requirements

- **CPU**: Intel i5 or better (i7 recommended)
- **RAM**: 8GB minimum (16GB recommended)
- **GPU**: NVIDIA GPU with 4GB+ VRAM (optional)
- **Storage**: 20GB free space

---

## 🔧 Installation

### 1. Install ROS2 Humble

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

### 2. Install Gazebo Harmonic

```bash
sudo apt update
sudo apt install gz-harmonic -y
```

### 3. Install ROS-Gazebo Bridge

```bash
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-image -y
```

### 4. Install PX4 Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gz_x500
```

### 5. Install Python Dependencies

```bash
pip3 install ultralytics opencv-python torch torchvision mavsdk matplotlib --break-system-packages
```

### 6. Clone This Repository

```bash
cd ~
git clone https://github.com/Omarm3h/drone-face-detection.git
cd drone-face-detection
```

### 7. Setup ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_yolo_ws/src
cd ~/ros2_yolo_ws/src

# Copy package
cp -r ~/drone-face-detection/ros2_workspace/src/yolo_face_detection .

# Build
cd ~/ros2_yolo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 8. Install Project Files

```bash
# Copy world to PX4
cp ~/drone-face-detection/worlds/boulevard.sdf \
   ~/PX4-Autopilot/Tools/simulation/gz/worlds/

# Copy model to PX4
cp -r ~/drone-face-detection/models/x500 \
      ~/PX4-Autopilot/Tools/simulation/gz/models/

# Set environment variable
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/drone-face-detection/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Usage

### Quick Start

#### Terminal 1: Launch PX4 + Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=boulevard PX4_GZ_MODEL_POSE="-50,0,0.3,0,0,0" make px4_sitl gz_x500
```

Wait for: `INFO  [commander] Ready for takeoff!`

#### Terminal 2: Launch Detection System

```bash
source ~/ros2_yolo_ws/install/setup.bash
ros2 launch yolo_face_detection boulevard_mission_no_gz.launch.py
```

Wait for: `[mission_face_detector_node]: YOLO loaded on cpu`

#### Terminal 3: Run Mission

```bash
source ~/ros2_yolo_ws/install/setup.bash
cd ~/drone-face-detection/scripts
python3 zigzag_mission.py
```

#### Terminal 4: View Detections (Optional)

```bash
source ~/ros2_yolo_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select `/face_detection/visualization` from dropdown.

---

## ⚙️ Configuration

### Detection Parameters

Edit launch file to adjust detection:

```bash
nano ~/ros2_yolo_ws/src/yolo_face_detection/launch/boulevard_mission_no_gz.launch.py
```

**Key parameters**:
- `model_path`: YOLO model (`yolov8n.pt`, `yolov8s.pt`, `yolov8m.pt`)
- `confidence_threshold`: Detection confidence (0.0-1.0)
- `device`: Processing device (`cpu` or `cuda`)

Example:
```bash
ros2 launch yolo_face_detection boulevard_mission_no_gz.launch.py \
  model_path:=yolov8m.pt \
  confidence_threshold:=0.6 \
  device:=cuda
```

### Mission Parameters

Edit mission script:

```bash
nano ~/drone-face-detection/scripts/zigzag_mission.py
```

**Key parameters**:
```python
SEARCH_ALT = 40.0            # Search altitude (meters)
TRACK_ALT = 40.0             # Tracking altitude (meters)
TRACK_DURATION_S = 60.0      # Tracking duration (seconds)
ZIGZAG_WIDTH_M = 120.0       # Search area width (meters)
ZIGZAG_LENGTH_M = 200.0      # Search area length (meters)
CRUISE_SPEED_MPS = 5.0       # Flight speed (m/s)
```

---

## 📁 Project Structure

```
drone-face-detection/
├── worlds/
│   └── boulevard.sdf                    # Gazebo world with urban environment
├── models/
│   └── x500/
│       ├── model.sdf                    # Drone model with camera
│       └── model.config                 # Model metadata
├── scripts/
│   └── zigzag_mission.py                # Autonomous mission script
├── ros2_workspace/
│   └── src/
│       └── yolo_face_detection/
│           ├── yolo_face_detection/
│           │   ├── face_detector_node.py
│           │   ├── mission_face_detector_node.py
│           │   └── camera_publisher_node.py
│           ├── launch/
│           │   └── boulevard_mission_no_gz.launch.py
│           ├── config/
│           ├── package.xml
│           └── setup.py
├── docs/
│   ├── INSTALLATION.md
│   ├── TROUBLESHOOTING.md
│   └── API.md
├── images/
│   └── demo.gif
├── .gitignore
├── LICENSE
└── README.md
```

---

## 📊 Mission Workflow

1. **Takeoff**: Drone arms and ascends to 40m altitude
2. **Transit**: Flies to boulevard center (250m east)
3. **Search**: Executes 6-row zigzag pattern over 120x200m area
4. **Detection**: Computer vision identifies people in frame
5. **Tracking**: Follows detected target for 60 seconds
6. **RTL**: Returns to launch point
7. **Landing**: Autonomous landing at runway
8. **Logging**: Saves flight path and detection data

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Raw camera feed |
| `/face_detection/detected` | std_msgs/Bool | Detection status |
| `/face_detection/position` | geometry_msgs/Point | Normalized position (x,y) |
| `/face_detection/count` | std_msgs/Int32 | Number of detections |
| `/face_detection/detections` | vision_msgs/Detection2DArray | Full detection data |
| `/face_detection/visualization` | sensor_msgs/Image | Annotated image |

---

## 🐛 Troubleshooting

### Camera Not Publishing

```bash
# Check Gazebo topics
gz topic -l | grep camera

# Verify bridge
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

### No Detections

- Ensure drone is over boulevard area (x ≈ 250m)
- Lower confidence threshold: `confidence_threshold:=0.3`
- Check camera view in rqt_image_view

### PX4 Connection Issues

```bash
# Check PX4 running
ps aux | grep px4

# Verify MAVLink port
netstat -an | grep 14540
```

### Performance Issues

- Use GPU: `device:=cuda`
- Use smaller model: `model_path:=yolov8n.pt`
- Reduce camera resolution in model.sdf

For more issues, see [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

---

## 🎓 Documentation

- **[Installation Guide](docs/INSTALLATION.md)**: Detailed setup instructions
- **[API Documentation](docs/API.md)**: ROS2 interface details
- **[Troubleshooting](docs/TROUBLESHOOTING.md)**: Common issues and solutions

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **PX4 Autopilot Team**: For the excellent flight controller
- **Tuwaiq Academy** — for technical training, structured support, and enabling applied learning in advanced technologies.
- **Ultralytics**: For YOLOv8 object detection
- **ROS2 Community**: For the robotics middleware
- **Gazebo Team**: For the simulation platform
  

---

## 📧 Contact

**Omarm3h** - - Omarm3h@icloud.com

---



## 📈 Future Enhancements

- [ ] Multi-drone coordination
- [ ] Face recognition (identification)
- [ ] Real-time path planning
- [ ] Obstacle avoidance
- [ ] Hardware deployment (Jetson Nano)
- [ ] Web-based monitoring dashboard
- [ ] Machine learning model fine-tuning

---

**Made  by [Omarm3h]**
