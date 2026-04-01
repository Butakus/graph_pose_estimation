# graph-pose-estimation

A pose estimation system for ROS 2 based on landmark detections and graph optimization.

The repository is organized as a multi-package ROS 2 project with:
- Message definitions for landmark detections and arrays.
- Core estimation and assignment libraries.
- Landmark file I/O utilities.
- An offline estimation executable for CSV/YAML data.

## Repository structure

### `gpe_msgs`
ROS 2 interface package with custom message definitions for 2D/3D landmarks and landmark detections.

Messages:
- `Landmark2D.msg`
- `Landmark2DArray.msg`
- `Landmark3D.msg`
- `Landmark3DArray.msg`
- `LandmarkDetection.msg`
- `LandmarkDetectionArray.msg`

### `gpe_core`
Core estimation package with reusable C++ libraries.

Contents:
- Libraries:
	- `gpe_types`: Shared data structures and type aliases used by the core algorithms.
	- `gpe_utils`: Utility helpers for common math and other operations.
	- `gpe_hungarian`: Hungarian algorithm implementation for landmark/detection association.
	- `gpe_se2_pose_estimation`: Main SE(2) graph-based pose estimation algorithm based on G2O.

### `gpe_landmark_server`
Landmark data loading/saving utilities.

Contents:
- Library:
	- `gpe_landmark_io`: Reads and writes landmark datasets from YAML files.

### `gpe_offline_estimation`
Offline pipeline for running SE(2) pose estimation from files.

Contents:
- Library:
	- `file_io`: File parsing and loading helpers for offline measurement and landmark inputs.
- Executable:
	- `se2_offline_estimation_node`


## Prerequisites

- ROS 2 installed and sourced. Main branch targets the `rolling` ROS 2 distro.
- `colcon` and `rosdep` installed.

## Build Instructions (rosdep + colcon)

From the workspace root (the folder that contains `src/`):

1. Install package dependencies with `rosdep`.

```bash
cd /path/to/your/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

2. Build with `colcon`.

```bash
colcon build --symlink-install
```

## Running Tests

Run tests for this repository:

```bash
colcon test --packages-select gpe_msgs gpe_core gpe_landmark_server gpe_offline_estimation
colcon test-result --verbose
```

## Running the Offline Estimation Executable

After building and sourcing the workspace:

```bash
ros2 run gpe_offline_estimation se2_offline_estimation_node
```

Use `--ros-args -p <name>:=<value>` to pass any node parameters expected by the executable.

For package-specific inputs, file formats, and test assets, see [gpe_offline_estimation/README.md](gpe_offline_estimation/README.md).

## License
This project is licensed under the GNU Lesser General Public License v3.0 (LGPLv3).
See the [LICENSE](LICENSE) file for details.
