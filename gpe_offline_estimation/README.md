# gpe_offline_estimation

Offline SE(2) pose estimation package for running graph-based estimation from files.

## Command-Line Arguments

The node uses positional command-line arguments (not ROS parameters) for input paths:

```bash
ros2 run gpe_offline_estimation se2_offline_estimation_node -- \
	<landmarks_file> <measurements_dir>
```

Arguments:
- `<landmarks_file>`: Path to the landmark YAML file.
- `<measurements_dir>`: Path to a directory containing `poses.csv` and `measurements_<timestamp>.csv` files.

Notes:
- `-h` prints usage and exits.
- Estimated output is written to `<measurements_dir>/output_poses.csv`.

## Expected Data Files

### 1) Landmark YAML (`<landmarks_file>`)

Expected top-level keys:
- `frame_id`: String frame name used for all landmarks.
- `landmarks`: Sequence of landmark entries.

Each landmark entry must include:
- `id`: Non-negative integer, unique in the file.
- `type`: `PointXY` or `Pose2D`.
- `coords`: Numeric list matching type size.

Coordinate sizes:
- `PointXY` -> `[x, y]`
- `Pose2D` -> `[x, y, theta]`

Example:

```yaml
frame_id: map
landmarks:
	- id: 1
		type: PointXY
		coords: [1.0, 2.0]
	- id: 2
		type: Pose2D
		coords: [3.5, -0.5, 1.57]
```

### 2) Poses CSV (`<measurements_dir>/poses.csv`)

One pose per line with 4 fields:

```text
<timestamp>;<x>;<y>;<theta>
```

Example:

```text
1717000001;0.0;0.0;0.0
1717000002;0.2;0.1;0.03
```

### 3) Measurement CSV files (`<measurements_dir>/measurements_<timestamp>.csv`)

The directory is scanned non-recursively for files named `measurements_<timestamp>.csv`.
Each line contains one landmark detection with 7 fields:

```text
<id>;<x>;<y>;<cov00>;<cov01>;<cov10>;<cov11>
```

Example:

```text
1;1.20;0.35;0.05;0.00;0.00;0.05
2;2.10;-0.40;0.08;0.01;0.01;0.07
```

Important:
- The `<timestamp>` in each measurement filename should match a timestamp present in `poses.csv`.
- Files with no matching pose timestamp are skipped.
- Input parser expects plain numeric rows (no header row).

## Run

After sourcing your workspace:

```bash
ros2 run gpe_offline_estimation se2_offline_estimation_node
```

With explicit input arguments:

```bash
ros2 run gpe_offline_estimation se2_offline_estimation_node -- \
	/path/to/test_landmarks.yaml /path/to/measurements
```
