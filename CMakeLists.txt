cmake_minimum_required(VERSION 3.8)
project(corgi_sim)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(corgi_driver_pkg)

# Install resource directories
install(DIRECTORY 
  launch
  protos
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()