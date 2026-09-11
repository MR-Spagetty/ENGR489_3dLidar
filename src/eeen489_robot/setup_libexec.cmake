# CMake script to set up libexec symlinks for ROS2 console scripts
# This is called as part of the build process to create symlinks that ROS2 launch expects

# Try to find the install directory
# When using symlink-install, we need to find where things are installed
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()

# Paths
set(BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(LIBEXEC_DIR "${CMAKE_INSTALL_PREFIX}/lib/eeen489_robot")

# List of console scripts
set(SCRIPTS
  low_level_ros_interface_node
  high_level_ros_interface_node
  dual_imu_publisher_node
  lidar_3d_pointcloud_node
  swap_scan_axis
)

# Only proceed if bin directory exists
if(EXISTS "${BIN_DIR}")
  # Create libexec directory if it doesn't exist
  file(MAKE_DIRECTORY "${LIBEXEC_DIR}")
  
  # Create symlinks from bin to libexec
  foreach(SCRIPT ${SCRIPTS})
    set(BIN_PATH "${BIN_DIR}/${SCRIPT}")
    if(EXISTS "${BIN_PATH}")
      set(LINK_PATH "${LIBEXEC_DIR}/${SCRIPT}")
      
      # Remove existing symlink/file if it exists
      if(EXISTS "${LINK_PATH}" OR IS_SYMLINK "${LINK_PATH}")
        file(REMOVE "${LINK_PATH}")
      endif()
      
      # Create relative symlink
      file(RELATIVE_PATH RELPATH "${LIBEXEC_DIR}" "${BIN_PATH}")
      
      # Use execute_process to create the symlink
      execute_process(
        COMMAND ln -s "${RELPATH}" "${LINK_PATH}"
        OUTPUT_QUIET
        ERROR_QUIET
      )
      
      if(EXISTS "${LINK_PATH}" OR IS_SYMLINK "${LINK_PATH}")
        message(STATUS "Created symlink: ${SCRIPT}")
      endif()
    endif()
  endforeach()
  
  message(STATUS "Libexec setup complete: ${LIBEXEC_DIR}")
else()
  message(WARNING "Bin directory not found: ${BIN_DIR}")
endif()
