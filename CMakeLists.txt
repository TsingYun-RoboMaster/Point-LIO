cmake_minimum_required(VERSION 3.10)
project(point_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

option(BUILD_MID360_SUPPORT "Build with Livox MID360 support" OFF)

# OpenMP
find_package(OpenMP QUIET)
if(OpenMP_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

if(BUILD_MID360_SUPPORT)
  find_package(livox_ros_driver2 REQUIRED)
  add_definitions(-DMID360_SUPPORT)
endif()

ament_auto_add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)

target_include_directories(pointlio_mapping PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(pointlio_mapping
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

set(AMENT_DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  pcl_conversions
)

if(BUILD_MID360_SUPPORT)
  list(APPEND AMENT_DEPENDENCIES livox_ros_driver2)
endif()

ament_target_dependencies(pointlio_mapping ${AMENT_DEPENDENCIES})

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
