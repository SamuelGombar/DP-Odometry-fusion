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

Implementované, spúšťané a testované na uvedených verziách.
Netestované na iných verziách.

---

## Dependencie

### ROS 2 Jazzy

Sú potrebné nasledovné dependencie. Ak máte inú distribúciu ros2, je ju potrebné zmeniť, napr. "jazzy" -> "humble"

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

---

## Build

```bash
mkdir -p ~/<workspace> && cd ~/<workspace>
git clone git@github.com:SamuelGombar/DP-Odometry-fusion.git
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Balíky použité v DP-Odometry-Fusion

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

## Vykreslovanie grafov a trajektórií

Na vykreslovanie grafov a trajekórií slúži skript run_trajectory_eval.sh v priečinku ~/<workspace>/start_scripts. Predtým je ale potrebné pripraviť virtuálne prostredie venv a nainštalovať potrebné balíky install numpy matplotlib pandas scipy. Stačí jednoducho spustiť setup_venv.sh.

```
cd ~/<workspace>/start_scripts
chmod +x setup_venv.sh
./setup_venv.sh
```

Sú potrebné nastaviť konfigurácie v skripte run_trajectory_eval.sh.

```
WORKSPACE_PATH=~/<workspace>    # cesta k pracovnému priečinku
BAG_NAME="Tetragon_4m"          # meno datasetu
KOBUKI=false                    # ak kobuki alebo autopicker
MODE="temporal"                 # mód vyhodnocovania chyby poloha - gt_poloha
SAVE_DIR="~/Desktop/temp"       # priečinok uloženia grafov
SAVE_PREFIX="one"               # sufix pre názvy, napr. one_plot.png
```

Sú vykreslované offline - priamo vyčítané z bagov.

---

## Spúšťanie nahratých datasetov

Na nasledovnom linku sa nachádzajú datasety pre nahraté fúzie vo forme ros2 bag.

```
https://drive.google.com/drive/folders/1PvXUu1KdgURwsZjl1HbctmSI2fn-UjsG?usp=sharing
```

po stiahnutí a rozbalení priečinka recordings ho je potrebné presunúť do pracovného priečinka (workspace ~/<workspace>):

```
mv ~/Downloads/recordings ~/<workspace>
```

Pre prehratie nahratých datasetov slúži skript play_bag.sh v priečinku ~/<workspace>/start_scripts.
Najprv je potrebné nastaviť konfiguráciu v play_bag.sh.

```
WORKSPACE_PATH=~/<workspace>    # cesta k pracovnému priečinku
BAG_NAME="Tetragon_4m"          # meno datasetu
KOBUKI=false                    # ak kobuki alebo autopicker
```

Pred spustením je potrebné udeliť práva na spúšťanie. Vizualizácia prebieha v Rviz2.

```
chmod +x play_bag.sh
./play_bag.sh
```

Ak chcete znížiť rýchlosť prehrávania bagu, je možné nastaviť pomocou ros2 service
uvedeného na konci play_bag.sh, alebo manuálne šípkami počas prehrávania bagu.

---