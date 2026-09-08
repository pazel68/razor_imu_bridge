# razor_imu_bridge

ROS 2 Python package that reads accelerometer and gyroscope data from a SparkFun Razor 9DoF IMU over a serial connection, converts the measurements to standard ROS units, applies static offsets, and publishes `sensor_msgs/msg/Imu` messages. The included launch file also starts `imu_filter_madgwick` to estimate orientation.

## Features

- Reads comma-separated IMU measurements from a serial port at 115200 baud by default.
- Converts acceleration from `g` to `m/s^2`.
- Converts angular velocity from degrees per second to `rad/s`.
- Applies configurable accelerometer and gyroscope offsets.
- Publishes raw IMU data on `/imu/data_raw` with frame ID `imu_link`.
- Includes a Madgwick-filter launch configuration.
- Includes a script and sample ROS 2 bag for calculating static offsets.

## Serial data format

The node expects at least 10 comma-separated fields on each line. Fields 1 through 6 (zero-based indexing) are used as follows:

```text
<prefix>,ax,ay,az,gx,gy,gz,<field7>,<field8>,<field9>
```

- `ax`, `ay`, `az`: acceleration in `g`
- `gx`, `gy`, `gz`: angular velocity in degrees per second

Malformed or incomplete lines are ignored.

## Requirements

- ROS 2 (Ubuntu 22.04 / ROS 2 Humble is the target environment)
- Python 3
- `pyserial`
- `imu_filter_madgwick`
- `rosbags` and ROS 2 bag Python libraries for the calibration script

Install the main runtime dependencies on ROS 2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-imu-filter-madgwick python3-serial
```

## Build

Clone the package into a ROS 2 workspace and build it:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/pazel68/razor_imu_bridge.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select razor_imu_bridge
source install/setup.bash
```

## Serial-port access

Connect the IMU and find its device path:

```bash
ls -l /dev/serial/by-id/
```

If the current user cannot access the device, add the user to the `dialout` group, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

## Run

Start the IMU bridge and Madgwick filter together:

```bash
ros2 launch razor_imu_bridge imu_filter.launch.py
```

Run only the serial bridge and override the port or offsets:

```bash
ros2 run razor_imu_bridge razor_imu_node --ros-args \
  -p port:=/dev/serial/by-id/<your-device-id> \
  -p baudrate:=115200 \
  -p accel_x_offset:=-0.155379 \
  -p accel_y_offset:=0.053448 \
  -p accel_z_offset:=0.030605 \
  -p gyro_x_offset:=-0.000822 \
  -p gyro_y_offset:=-0.001320 \
  -p gyro_z_offset:=-0.000517
```

The launch file currently contains the calibrated offsets used by the target robot. Adjust them for a different IMU installation.

## ROS interface

### Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Converted acceleration and angular velocity without an orientation estimate |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation estimate published by `imu_filter_madgwick` when using the launch file |

### Parameters

| Parameter | Default | Unit / description |
| --- | --- | --- |
| `port` | SparkFun device path in the node | Serial device path |
| `baudrate` | `115200` | Serial baud rate |
| `accel_x_offset` | `0.0` | X acceleration offset in `m/s^2` |
| `accel_y_offset` | `0.0` | Y acceleration offset in `m/s^2` |
| `accel_z_offset` | `0.0` | Z acceleration offset in `m/s^2` |
| `gyro_x_offset` | `0.0` | X angular-velocity offset in `rad/s` |
| `gyro_y_offset` | `0.0` | Y angular-velocity offset in `rad/s` |
| `gyro_z_offset` | `0.0` | Z angular-velocity offset in `rad/s` |

Inspect the output:

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data
```

## Calculate static offsets

Keep the IMU stationary on a level surface and record `/imu/data_raw`:

```bash
ros2 bag record /imu/data_raw -o imu_calibration_bag
```

Stop the recording after collecting enough stationary samples, then run:

```bash
python3 src/razor_imu_bridge/scripts/calc_imu_offset.py imu_calibration_bag
```

The script prints six offset parameters. Copy those values into `launch/imu_filter.launch.py` or pass them to the node with `--ros-args -p`.

## Troubleshooting

- **Cannot open the serial port:** verify the device path, cable, and `dialout` membership.
- **No messages on `/imu/data_raw`:** check that the serial stream contains at least 10 comma-separated fields and uses the expected units.
- **Orientation drifts:** keep the IMU stationary during calibration and tune the Madgwick settings in `config/imu_filter.yaml`.
- **Wrong axes or signs:** confirm the physical IMU mounting and transform conventions for `imu_link`.

## License

No license has been declared for this repository yet.
