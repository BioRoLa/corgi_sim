cmake_minimum_required(VERSION 3.8)
project(corgi_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)

# Install Python package using setup.py
ament_python_install_package(corgi_driver_pkg)

# Note: setup.py will handle:
# - Python scripts installation
# - Entry points
# - Data files (launch, worlds, protos, etc.)

ament_package()