cmake_minimum_required(VERSION 3.8)
project(eeen489_robot)

# Find ament_cmake_python and other required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Set Python install directory
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

