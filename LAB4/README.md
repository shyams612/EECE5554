# LAB4 — Dead Reckoning Navigation with IMU and Magnetometer

**Driver:** Shyam Sreenivasan

---

## Hardware

| Device | Default port |
|---|---|
| GPS puck (GPGGA NMEA) | `/dev/ttyUSB0` |
| VectorNav VN-100 IMU | `/dev/ttyUSB1` |

Confirm which USB port each device is on before running:

```bash
ls /dev/ttyUSB*
# Plug/unplug each device to identify which is which
```

---

## Build

```bash
cd LAB4
colcon build --packages-select msgs gps_driver imu_driver
source install/setup.bash
```

Build `msgs` first — the drivers depend on the custom message types it defines (`GpsMsg`, `IMUmsg`).

---

## Run

### Option A — Both drivers together (recommended)

Launches the GPS driver and IMU driver in a single command.

```bash
ros2 launch imu_driver combined_launch.py \
    gps_port:=/dev/ttyUSB0 \
    imu_port:=/dev/ttyUSB1
```

### Option B — Drivers separately (two terminals)

**Terminal 1 — GPS driver:**

```bash
ros2 run gps_driver driver --ros-args -p /dev/ttyUSB0
```

**Terminal 2 — IMU driver:**

```bash
ros2 run imu_driver imu_driver /dev/ttyUSB1
```

### Option C — IMU driver only

```bash
ros2 launch imu_driver imu_launch.py port:=/dev/ttyUSB1
```

---

## Record a ROS Bag

Open a **new terminal** while the drivers are running and source the workspace:

```bash
source install/setup.bash
```

Then record both topics:

```bash
ros2 bag record /gps /imu -o <bag_name>
```

Replace `<bag_name>` with a descriptive name, e.g. `circle_bag` or `adventure_bag`.

The bag will be saved as a directory `<bag_name>/` containing a `.db3` SQLite file and a `metadata.yaml`.

**Stop recording:** `Ctrl+C`

---

## Verify the Bag

List recorded topics and message counts:

```bash
ros2 bag info <bag_name>
```

Echo a few messages to confirm data is being captured:

```bash
ros2 bag play <bag_name> --loop &
ros2 topic echo /gps --once
ros2 topic echo /imu --once
```

---

## Published Topics

| Topic | Message type | Rate | Content |
|---|---|---|---|
| `/gps` | `msgs/msg/GpsMsg` | ~1 Hz | Latitude, longitude, altitude, HDOP, UTM easting/northing, UTC, zone |
| `/imu` | `msgs/msg/IMUmsg` | 40 Hz | Quaternion, gyro (rad/s), accel (m/s²), magnetometer (Gauss), raw VNYMR string |

---

## Analysis

After collecting bags, copy them into `data/` and run the analysis pipeline:

```bash
cd analysis
python main.py
```

Results and plots are written to `analysis/results/`.
Edit `analysis/config.json` to point to different bag files or adjust filter parameters.
