# Copilot Coding Agent Instructions — DP-Odometry-Fusion

## Repository Summary

ROS 2 (Jazzy) workspace for mobile robot localization and odometry research. Compares multiple scan-matching / sensor-fusion approaches (CSM ICP, GenZ-ICP, EKF/UKF) on a Kobuki robot platform. Runs on **Ubuntu 24.04** with **ROS 2 Jazzy**, built with **colcon** and **ament_cmake**.

**Languages:** C++ (C++14/17/20 depending on package), Python 3.12  
**Build tools:** colcon 0.20, CMake 3.28, g++ 13.3  
**No CI/CD pipelines or GitHub workflows exist.** No Dockerfiles.

---

## Build Instructions

### Environment Setup (always do this first in every terminal)

```bash
source /opt/ros/jazzy/setup.bash
```

### Build All Packages

```bash
cd ~/ros2_ws_host
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Full clean build takes ~2 minutes. Incremental builds are fast (<1 s per package).

### Build a Single Package

```bash
colcon build --symlink-install --packages-select <package_name>
```

To also build its dependencies:

```bash
colcon build --symlink-install --packages-up-to <package_name>
```

### Source Workspace After Building (required to run nodes)

```bash
source install/setup.bash
```

### Known Build Issue — Symlink Errors on Message Packages

After switching branches or on stale builds, message packages (`alert_msgs`, `amrapi_msgs`, `hw_layer_msgs`, `robot_localization`) may fail with:

```
failed to create symbolic link '…/ament_cmake_python/<pkg>/<pkg>' because existing path cannot be removed: Is a directory
```

**Fix:** Delete the offending build directory and rebuild:

```bash
rm -rf build/<failing_package>
colcon build --symlink-install
```

If multiple message packages fail, remove all their build dirs at once:

```bash
rm -rf build/alert_msgs build/amrapi_msgs build/hw_layer_msgs build/robot_localization
colcon build --symlink-install
```

### Running Tests

Only `robot_localization` has tests. Run them with:

```bash
colcon test --packages-select robot_localization
colcon test-result --all
```

Two integration tests (`test_ekf_localization_node_interfaces`, `test_ukf_localization_node_interfaces`) are known to produce errors in this workspace. Unit tests (filter_base, test_ekf, test_ukf) and lint tests pass.

---

## Package Dependency Graph

Build order matters. The dependency chain is:

```
alert_msgs          (no deps — build first)
├── amrapi_msgs     (depends on alert_msgs)
│   └── robot_control_cpp (depends on amrapi_msgs, hw_layer_msgs)
hw_layer_msgs       (no deps)
csm                 (no deps)
├── ros2_laser_scan_matcher (depends on csm)
genz_icp            (no deps — fetches core lib via CMake FetchContent)
robot_localization  (no deps in workspace)
icp_localization_ros2 (no deps in workspace — needs libpointmatcher system-installed)
uds_kobuki_ros      (no deps in workspace — Python package)
```

---

## Project Layout

### Root Files

| File | Purpose |
|---|---|
| `commands.txt` | Reference ROS 2 bag/TF commands |
| `README.md` | One-line project title |
| `TODO.txt` / `PRED_ZAPNUTIM.txt` | Developer notes (Slovak), git-ignored |
| `.gitignore` | Ignores `build/`, `install/`, `log/`, `recordings/`, `.vscode/`, `*.pdf` |

### Source Packages (`src/`)

| Package | Type | C++ Std | Description |
|---|---|---|---|
| `alert_msgs` | ament_cmake (msgs) | — | Single `RobotAlert.msg` definition |
| `amrapi_msgs` | ament_cmake (msgs) | — | 30 message types for AMR API |
| `hw_layer_msgs` | ament_cmake (msgs) | — | 20 hardware-layer sensor messages |
| `csm` | ament_cmake (lib) | C++14 | Canonical Scan Matcher library (ICP) |
| `ros2_laser_scan_matcher` | ament_cmake | C++14 | CSM-based laser odometry node |
| `genz-icp` | ament_cmake | C++20 | GenZ-ICP degeneracy-robust LiDAR odometry |
| `robot_control_cpp` | ament_cmake | C++14 | 10 utility nodes (odom, teleop, scan conversion, etc.) |
| `robot_localization` | ament_cmake | C++17 | EKF/UKF sensor fusion (upstream fork) |
| `icp_localization_ros2` | ament_cmake | C++14 | ICP localization in pre-built maps |
| `uds_kobuki_ros` | ament_python | — | Kobuki robot driver (UDP-based) |

### Key Directories

- **`src/<pkg>/launch/`** — ROS 2 launch files (Python `.launch.py`)
- **`src/robot_localization/params/`** — EKF/UKF YAML parameter files (especially `ekf.yaml`)
- **`src/genz-icp/ros/config/`** — GenZ-ICP parameter presets per environment
- **`src/icp_localization_ros2/config/`** — ICP and sensor filter YAML configs
- **`rviz/`** — 7 RViz visualization configs for different benchmarking scenarios
- **`start_scripts/`** — 8 shell scripts for launching node combinations and benchmarks
- **`recordings/`** — ROS 2 bag files (git-ignored, not in repo)

### Main Executables by Package

**robot_control_cpp** (10 nodes): `teleop`, `pid_control`, `scan_to_pc`, `odom_republisher`, `scan_republisher`, `wheel_odom_publisher`, `odom_to_path`, `imu_publisher`, `groundtruth_republisher`, `gt_wrapper`

**robot_localization** (4 nodes): `ekf_node`, `ukf_node`, `navsat_transform_node`, `robot_localization_listener_node`

**ros2_laser_scan_matcher** (1 node): `laser_scan_matcher`

**genz_icp** (1 node): `odometry_node` (also available as composable component `odometry_component`)

**uds_kobuki_ros** (1 node): `uds_kobuki_ros`

### Code Style

- `genz-icp` has a `.clang-format` (Google-based, 4-space indent, 100-col limit, C++17) and `.cmake-format.yaml`
- `robot_localization` runs ament linters in tests: `cppcheck`, `cpplint`, `uncrustify`, `flake8`, `pep257`
- Other packages have no enforced style — follow existing conventions in each package

---

## Useful Commands

```bash
# List workspace packages
colcon list

# Show dependency graph
colcon graph

# Run a specific node
ros2 run <package_name> <executable_name>

# Launch with parameters
ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic
ros2 launch robot_localization ekf.launch.py params_file:=<path_to_ekf.yaml>

# Play recorded bag data for testing
ros2 bag play <path_to_bag> --topics /scan_merged /amrapi/sensor/velocity /hw_layer/imu/sensor/data
```

---

## Important Notes

- Always run `source /opt/ros/jazzy/setup.bash` before any `colcon` or `ros2` command.
- Always run `source install/setup.bash` after building, before running nodes.
- The `genz_icp` package fetches its C++ core library via CMake `FetchContent` from GitHub on first build — internet access is required for the initial build.
- The `icp_localization_ros2` package depends on `libpointmatcher` being installed system-wide (not in this workspace).
- When adding new messages to `*_msgs` packages, regenerate by rebuilding that package (delete its `build/` dir first if using `--symlink-install`).
- The `uds_kobuki_ros` `setup.cfg` uses deprecated dash-separated keys (`install-scripts`); this produces a warning but does not break the build.
- Trust these instructions. Only search for additional build/config information if the instructions above are incomplete or found to be incorrect.
