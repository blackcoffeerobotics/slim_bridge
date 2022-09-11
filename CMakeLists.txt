cmake_minimum_required(VERSION 3.5)

project(slim_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# find ROS 1 packages
set(cmake_extras_files cmake/find_ros1_package.cmake)
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(WARNING "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  if(NOT ROS1_PACKAGES_UNAVAILABLE)
    # Only warn if ROS1 packages were expected to be available
    message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  endif()
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package(
    CONFIG_EXTRAS ${cmake_extras_files}
  )
  return()
endif()

find_ros1_package(std_msgs REQUIRED)
find_ros1_package(tf2_msgs REQUIRED)
find_ros1_package(geometry_msgs REQUIRED)
find_ros1_package(sensor_msgs REQUIRED)
find_ros1_package(nav_msgs REQUIRED)

set(SLIM_BRIDGE_DEPENDS
  rclcpp
  ros1_roscpp
  std_msgs
  ros1_std_msgs
  yaml_cpp_vendor
  sensor_msgs
  ros1_sensor_msgs
  tf2_msgs
  ros1_tf2_msgs
  nav_msgs
  ros1_nav_msgs
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package(
  CONFIG_EXTRAS ${cmake_extras_files}
)

include_directories(include/)

add_executable(bridge src/bridge.cpp)
ament_target_dependencies(bridge ${SLIM_BRIDGE_DEPENDS})
target_include_directories(bridge PUBLIC include)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY cmake launch config
  DESTINATION share/${PROJECT_NAME}
)