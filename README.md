# Fúzia kolesovej odometrie a LiDAR odometrie 

Repozitár implementácie diplomovej práce. Slúži na porovnávanie fúzií GenZ-ICP + EKF, Kinematic-ICP a PLICP. Experimentálne vyhodnotenie je vykonané
na reálnom mobilnom robote Kobuki a Brightpick Autopicker.

---

## Požiadavky

| Požiadavka | Verzia |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| CMake | ≥ 3.28 |
| g++ | ≥ 13 |
| Python | 3.12 |

---

## Dependencie

### ROS 2 Jazzy

Sú potrebné nasledovné dependencie:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-common \
  ros-jazzy-nav2-util \
  ros-jazzy-nav2-msgs \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-eigen \
  ros-jazzy-laser-geometry \
  ros-jazzy-message-filters \
  ros-jazzy-geographic-msgs \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-rosbag2-cpp \
  ros-jazzy-rosbag2-storage \
  ros-jazzy-rosbag2-transport \
  ros-jazzy-rosbag2-interfaces \
  ros-jazzy-rosbag2-py \
  ros-jazzy-ros2bag \
  ros-jazzy-yaml-cpp-vendor \
  ros-jazzy-rclcpp-components \
  ros-jazzy-rviz2 \
  ros-jazzy-rclpy \
  ros-jazzy-rosidl-runtime-py
```

### C++ knižnice

Sú potrebné pre knižnice robot_localization a kinematic-icp.

```bash
sudo apt install -y \
  libboost-dev \
  libeigen3-dev \
  libtbb-dev \
  libgeographic-dev
```

### Python knižnice

Potrebné pre skripty na vyhodnotenie trajektórie (`trajectory_eval.py`, `plot_trajectory_eval.py`, `plot_error_over_time.py`):

```bash
pip install numpy matplotlib pandas scipy
```

---

## Build

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone git@github.com:SamuelGombar/DP-Odometry-fusion.git
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Balíky

| Balík | Popis |
|---|---|
| `alert_msgs` | Definície vlastných správ o upozorneniach |
| `amrapi_msgs` | Definície správ AMR API |
| `hw_layer_msgs` | Definície správ senzorov hardvérovej vrstvy |
| `csm` | Knižnica Canonical Scan Matcher (ICP) potrebná pre ros2_laser_scan_matcher |
| `ros2_laser_scan_matcher` | PLICP odometria |
| `genz-icp` | GenZ-ICP odometria |
| `kinematic-icp` | Kinematic-ICP fúzia |
| `robot_control_cpp` | Pomocné uzly: odometria, teleop, konverzia skenu, IMU |
| `robot_localization` | fúzia EKF použitá pre GenZ-ICP |
| `uds_kobuki_ros` | Ovládač robota Kobuki |

---

## Spúšťanie nahratých datasetov

Na nasledovnom linku sa nachádzajú datasety pre nahraté fúzie vo forme ros2 bag.

```
https://drive.google.com/drive/folders/1PvXUu1KdgURwsZjl1HbctmSI2fn-UjsG?usp=sharing
```

po stiahnutí a rozbalení priečinka recordings je potrebné ho presunúť do pracovného priečinka (workspace ~/ros2_ws):

```
mv ~/Downloads/recordings ~/ros2_ws
```

Pre prehratie nahratých datasetov slúži skript play_bag.sh v priečinku ~/ros2_ws/start_scripts.
Najprv je potrebné nastaviť konfiguráciu v play_bag.sh.

```
WORKSPACE=~/ros2_ws         # cesta k pracovnému priečinku
BAG_NAME="Tetragon_4m"      # meno datasetu
KOBUKI=false                # ak kobuki alebo autopicker
```

Pred spustením je potrebné udeliť práva na spúšťanie. Vizualizácia prebieha v Rviz2.

```
chmod +x play_bag.sh
./play_bag.sh
```
