cmake_minimum_required(VERSION 3.8)
project(gps_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpsMsg.msg"
  "msg/RtkMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install Python modules to where the messages are generated
install(
  DIRECTORY gps_driver/
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables  
install(PROGRAMS
  gps_driver/driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME driver
)

install(PROGRAMS
  gps_driver/rtk_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rtk_driver
)

ament_package()