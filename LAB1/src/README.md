# Source ros2
source /opt/ros/humble/setup.bash

# Colcon build
colcon build

# source the setup file
source install/setup.sh

# run the driver
ros2 launch gps_driver driver.launch.py port:=/dev/pts/3