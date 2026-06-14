cmake_minimum_required(VERSION 3.8)
project(mushr_pf)

# Compiler warnings (only relevant if C++ targets are added later)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Runtime dependencies (Python nodes + messages)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf_transformations REQUIRED)

# -------------------------------
# Python package + scripts
# -------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/viz_sensormodel.py
  scripts/plot_vs_ta_pf.py
  scripts/viz_resample.py
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Shared resources
# -------------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

# Export runtime dependencies for downstream packages
ament_export_dependencies(
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  vesc_msgs
  tf2_ros
  tf_transformations
)

ament_package()
