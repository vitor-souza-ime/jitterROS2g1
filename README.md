# ⚙️ jitterROS2g1

## 📌 Overview

`jitterROS2g1` is a ROS 2-based tool designed to measure and analyze **temporal jitter in robotic sensor streams** on the Unitree G1 EDU humanoid robot 🤖.

It focuses on timing variability in two critical perception sources:

- 📡 IMU data from `/lowstate` (`unitree_hg/LowState`)
- 🌐 LiDAR point clouds from `/utlidar/cloud_livox_mid360` (`sensor_msgs/PointCloud2`)

The tool provides detailed statistical analysis of:

- ⏱️ ROS 2 / DDS transport jitter (inter-arrival time)
- 📊 Sensor timestamp drift
- 🔁 End-to-end latency variability

---

## 🎯 Motivation

In humanoid robotics, **timing determinism is critical** for perception, control, and stability. However, ROS 2 introduces variability due to:

- 🧩 DDS middleware scheduling behavior  
- ⚙️ QoS policies (RELIABLE vs BEST_EFFORT)  
- 🖥️ Operating system scheduling and CPU contention  
- 📡 Sensor publishing frequency instability  

This project enables **reproducible real-hardware experiments** to quantify these effects on the Unitree G1 platform.

---

## ✨ Features

- 📡 IMU jitter extraction from `/lowstate`
- 🌐 LiDAR jitter analysis from Livox Mid-360
- ⏱️ Inter-arrival time measurement (ROS 2 transport jitter)
- 🧠 Sensor timestamp drift estimation
- 📊 Statistical metrics (mean, std, p50, p95, p99)
- ⚙️ QoS comparison (RELIABLE / BEST_EFFORT / VOLATILE)
- 🔄 Parallel IMU + LiDAR acquisition mode
- 💾 Automatic CSV export for offline analysis

---

## 🧠 Supported Sensors

### 📡 IMU (Unitree LowState)
- Accelerometer
- Gyroscope
- Orientation (roll, pitch, yaw)
- Quaternion
- Internal robot tick

### 🌐 LiDAR (Livox Mid-360)
- Point cloud (`PointCloud2`)
- Timestamp-based scan analysis
- Point density per scan

---

## 🖥️ System Requirements

- 🐧 Ubuntu 22.04+
- 🤖 ROS 2 (Jazzy recommended)
- 🐍 Python 3.10+
- 🧩 Unitree ROS 2 workspace (`unitree_ros2`)
- 📡 Livox ROS driver (Mid-360 support)

---

## ⚡ Installation

### 1. Clone repository

```bash
git clone https://github.com/vitor-souza-ime/jitterROS2g1.git
cd jitterROS2g1
````

### 2. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 3. Build workspace

```bash
colcon build
source install/setup.bash
```

---

## 🚀 Usage

### 📡 IMU jitter analysis only

```bash
python3 jitter_imu_lidar_ros2.py --duration 120 --skip-lidar --output ~/data/jitter
```

---

### 🌐 LiDAR jitter analysis only

```bash
python3 jitter_imu_lidar_ros2.py --duration 120 --skip-imu --output ~/data/jitter
```

---

### 🔄 IMU + LiDAR (parallel mode)

```bash
python3 jitter_imu_lidar_ros2.py --parallel --duration 120 --output ~/data/jitter
```

---

### 🎯 Custom LiDAR topic

```bash
python3 jitter_imu_lidar_ros2.py --lidar-topic /utlidar/cloud_livox_mid360
```

---

## 📊 Metrics Computed

### 📡 IMU

* ⏱️ `inter_recv` → ROS 2 transport jitter
* 🧠 `inter_tick` → robot internal tick jitter
* 📈 Acceleration, angular velocity, orientation

### 🌐 LiDAR

* ⏱️ `inter_recv` → transport jitter
* 📡 `inter_stamp` → sensor timestamp jitter
* 📦 Number of points per scan

---

## ⚙️ QoS Configuration

The tool evaluates different ROS 2 QoS policies:

* 🟥 **RELIABLE** → guaranteed delivery (higher latency)
* 🟨 **BEST_EFFORT** → lower latency (possible packet loss)
* 🟦 **VOLATILE** → no persistence

Used to study **latency vs reliability trade-offs** in DDS.

---

## 💾 Output

Generated files:

* `imu_jitter.csv`
* `lidar_jitter.csv`

Each record includes:

* timestamps ⏱️
* jitter values 📊
* sensor measurements 📡
* statistical data ready for analysis 🧠

---

## 📁 Project Structure

```bash
jitterROS2g1/
├── jitter_imu_lidar_ros2.py   # main script
├── README.md                  # documentation
├── data/                      # output CSVs
└── results/                   # analysis outputs
```

---

## 📈 Applications

* 🤖 Real-time robotics benchmarking
* 📡 DDS / ROS 2 middleware evaluation
* 🧠 Humanoid perception timing analysis
* ⚙️ QoS impact studies
* 🎓 Academic research in robotic systems

---

## 📚 Citation

If you use this project in research:

```text
Vitor Amadeu Souza, "Temporal Jitter Characterization in ROS 2 Middleware for Humanoid Robotics", 2026.
```

---

## 📄 License

MIT License (or define your project license)

