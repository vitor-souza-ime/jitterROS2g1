# ⚙️ jitterROS2g1

ROS 2 timing measurements for the IMU and Livox Mid-360 point-cloud streams on the Unitree G1 EDU humanoid robot.

The repository contains a Python subscriber that records callback execution times and source-provided timestamps, calculates interval statistics, and exports the measurements to CSV.

## 📡 Topics and message types

| Sensor | ROS 2 topic | Message type |
|---|---|---|
| IMU | `/lowstate` | `unitree_hg/LowState` |
| LiDAR | `/utlidar/cloud_livox_mid360` | `sensor_msgs/PointCloud2` |

The LiDAR topic can be changed with `--lidar-topic`.

## 📊 Measurements

### IMU

- `inter_recv`: interval between consecutive subscriber callback execution timestamps, measured with `time.monotonic()`.
- `inter_tick`: difference between consecutive robot `tick` counter values. The counter is expressed in milliseconds and converted to seconds for the reported statistics.
- The CSV also stores accelerometer, gyroscope, orientation, and quaternion fields available in `LowState`.

### LiDAR

- `inter_recv`: interval between consecutive subscriber callback execution timestamps, measured with `time.monotonic()`.
- `inter_stamp`: difference between consecutive message `header.stamp` values.
- `n_points`: point count calculated from the `PointCloud2` message layout fields.

The callback intervals describe when the application processes messages. They are not hardware-level packet arrival timestamps or one-way network latency measurements. Likewise, differences between `header.stamp` values do not measure absolute clock offset or clock drift.

For each interval series, the program reports the sample count, minimum, maximum, mean, population standard deviation, p50, p95, and p99.

## ⚙️ QoS configuration

The current implementation uses fixed QoS profiles:

| Stream | Reliability | Durability | History | Depth |
|---|---|---|---|---:|
| IMU (`/lowstate`) | RELIABLE | VOLATILE | KEEP_LAST | 1 |
| LiDAR | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |

These are the profiles used by the subscribers; the program does not run a comparative QoS benchmark.

## 🖥️ Requirements

The measurement computer must have:

- Ubuntu with a compatible ROS 2 installation (the experimental environment used Ubuntu 24.04 and ROS 2 Jazzy).
- Python compatible with that ROS 2 installation.
- The `rmw_cyclonedds_cpp` RMW implementation.
- The Unitree ROS 2 message packages, including `unitree_hg`.
- Access to the G1 topics over the robot's Ethernet network.

The G1 must be publishing the selected topics. The Unitree workspace must already be built and its message definitions must be available to Python.

> **Workspace paths:** `main.py` contains default paths for the Unitree workspace under `/home/u7/unitree_ros2` and for ROS 2 Jazzy under `/opt/ros/jazzy`. If your installation uses different locations or a different ROS 2 distribution, adjust the paths in the environment setup section at the beginning of `main.py`.

## 🚀 Installation and execution

Clone the repository:

```bash
git clone https://github.com/vitor-souza-ime/jitterROS2g1.git
cd jitterROS2g1
```

Make sure ROS 2 and the Unitree message packages are available in the environment. For ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

For an external measurement computer, allow ROS 2 communication beyond the local host and select Cyclone DDS:

```bash
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Run the complete sequential measurement (IMU followed by LiDAR):

```bash
python3 main.py --duration 120 --output ~/data/jitter
```

The sequential mode runs the requested duration for each sensor, with a short pause between the IMU and LiDAR phases. Therefore, a duration of 120 seconds means approximately 120 seconds per sensor, not 120 seconds for the complete run.

### Run one sensor only

IMU:

```bash
python3 main.py --duration 120 --skip-lidar --output ~/data/jitter
```

LiDAR:

```bash
python3 main.py --duration 120 --skip-imu --output ~/data/jitter
```

### Run both sensors simultaneously

```bash
python3 main.py --duration 120 --parallel --output ~/data/jitter
```

### Select a different LiDAR topic

```bash
python3 main.py --duration 120 --skip-imu --lidar-topic /your/lidar/topic --output ~/data/jitter
```

## 💾 Output files

The program creates the output directory when necessary and writes:

- `imu_jitter.csv`
- `lidar_jitter.csv`

The CSV files contain the recorded timestamps, interval measurements, and sensor fields collected by the corresponding subscriber.

## ⚠️ Scope of the measurements

This tool characterizes timing intervals observed by ROS 2 subscriber callbacks and differences between consecutive source-provided timestamps. It does not independently measure:

- One-way network latency or end-to-end sensor-to-application latency.
- Absolute clock offset or drift between the robot and the measurement computer.
- Packet loss at the Ethernet or DDS transport layers.
- Comparative performance across different DDS middleware implementations or QoS profiles.
- The effect of timing variations on SLAM trajectory accuracy, map quality, or locomotion.

Those questions require additional instrumentation or experiments beyond the measurements implemented here.

## 📁 Repository structure

```text
jitterROS2g1/
├── main.py
└── README.md
```

CSV files are written to the directory supplied with `--output`.

## 📚 Citation

If you use this software in academic work, please cite the associated study:

Vitor Amadeu Souza, “Temporal Jitter Characterization of IMU and LiDAR via ROS 2 on the Unitree G1 EDU Humanoid Robot: Implications for Sensor Fusion and Real-Time Control,” 2026.

## 📄 License

No license file is currently specified in this repository. Add a license file if you intend to grant permissions for reuse and redistribution.
