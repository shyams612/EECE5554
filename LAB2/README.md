# cd into the root folder, say Lab1, or Lab2
cd LAB2

# Remove build install and log folders
rm -rf build/ log/ install/

# Source ros2
source /opt/ros/humble/setup.bash

# Colcon build
colcon build

# source the setup file
source install/setup.sh

# run the driver
ros2 launch gps_driver driver.launch.py port:=/dev/ttyUSB0

------------------------------------------
# cd into the sensor emulator dir in src/sensor_emulator. Run the emulator
python serial_emulator.py -f <full-path>/openRTK.txt -dev gps

# Echo to listen to the published messages
ros2 topic echo /rtk

# Capture messages in ros bag
ros2 bag record /rtk 

# Create comparative plots for gps (lab1) with rtk (lab2)
# Open area comparison
python analyze_gps2.py --gps ../data/gps_open_data --rtk ../data/rtk_open_data --scenario open

# Occluded area comparison
python analyze_gps2.py --gps ../data/gps_occluded_data --rtk ../data/rtk_occluded_data --scenario occluded

# Walking data comparison
python analyze_gps2.py --gps ../data/gps_walking_data --rtk ../data/rtk_walking_data --scenario walking