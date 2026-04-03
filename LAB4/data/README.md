# LAB4 — Dead Reckoning Navigation with IMU and Magnetometer

EECE 5554 — Robotics Sensing and Navigation  
Shyam Sreenivasan, Northeastern University

---

## Repository Structure

```
LAB4/
├── src/
│   ├── gps_driver/        # GPS driver node (from Lab 1)
│   ├── imu_driver/        # IMU driver node (from Lab 3)
│   └── msgs/              # Custom message definitions (GPSmsg, IMUmsg)
├── data/
│   └── *.db3              # ROS2 bag files
├── analysis/
│   ├── src/               # Analysis modules
│   │   ├── data_loader.py
│   │   ├── mag_calibration.py
│   │   ├── yaw_estimation.py
│   │   ├── velocity.py
│   │   └── dead_reckoning.py
│   ├── results/           # Generated plots and calibration data
│   ├── config.json        # Analysis configuration
│   ├── run_calibration.py
│   ├── run_yaw.py
│   ├── run_velocity.py
│   ├── run_dead_reckoning.py
│   └── Report.pdf
└── README.md
```

---

## Launching Both Driver Nodes

Both the GPS and IMU driver nodes can be launched simultaneously using the
provided launch file.

### Prerequisites

- ROS2 Humble installed and sourced
- Python packages: `pyserial`, `numpy`
- GPS puck connected (USB) — typically `/dev/ttyUSB0`
- VectorNav VN-100 connected (USB) — typically `/dev/ttyUSB1`

### Build

```bash
cd LAB4
colcon build
source install/setup.bash
```

### Launch both nodes

```bash
ros2 launch imu_driver combined_launch.py gps_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1
```

The `combined_launch.py` file is located at `src/imu_driver/launch/combined_launch.py`.
Default ports (`/dev/ttyUSB0` for GPS, `/dev/ttyUSB1` for IMU) can be overridden via the arguments above.

This launches:
- `/gps_driver` node publishing on `/gps` (`msgs/msg/GpsMsg`)
- `/imu_driver` node publishing on `/imu` (`msgs/msg/IMUmsg`)

### Verify topics are publishing

```bash
ros2 topic list
ros2 topic hz /gps     # should show ~1 Hz
ros2 topic hz /imu     # should show ~40 Hz
```

### Record a bag

```bash
ros2 bag record /gps /imu -o my_bag
```

---

## Running the Analysis

All analysis scripts are run from the `analysis/` directory:

```bash
cd LAB4/analysis
source ~/py3/bin/activate   # or your Python environment

# Chapter 1: Magnetometer calibration only
python run_calibration.py

# Chapter 1: Yaw estimation
python run_yaw.py

# Chapter 2: Forward velocity
python run_velocity.py

# Chapters 2 + 3: Full pipeline (calibration → yaw → velocity → dead reckoning)
python run_dead_reckoning.py
```

Results (plots + calibration params) are saved to `analysis/results/`.

---

## Driver Credits

GPS driver uploaded by: **Shyam Sreenivasan**  
IMU driver uploaded by: **Shyam Sreenivasan**
