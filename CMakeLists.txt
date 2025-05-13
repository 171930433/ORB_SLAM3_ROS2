cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(Pangolin REQUIRED)
find_package(orb_slam3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
)

# link_directories(
#   include
# )

# add_executable(mono
#   src/monocular/mono.cpp
#   src/monocular/monocular-slam-node.cpp
# )
# ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

# add_executable(rgbd
#   src/rgbd/rgbd.cpp
#   src/rgbd/rgbd-slam-node.cpp
# )
# ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(imu_rgbd
  src/imu_rgbd/imu_rgbd.cpp
  src/imu_rgbd/imu_rgbd-slam-node.cpp
)
target_link_libraries(imu_rgbd
orb_slam3::orb_slam3
)
ament_target_dependencies(imu_rgbd 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  message_filters 
  tf2_ros
  tf2_eigen
  nav_msgs
  Pangolin
)

# add_executable(stereo
#   src/stereo/stereo.cpp
#   src/stereo/stereo-slam-node.cpp
# )
# ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
# ORB_SLAM3 Pangolin)

# add_executable(stereo-inertial
#   src/stereo-inertial/stereo-inertial.cpp
#   src/stereo-inertial/stereo-inertial-node.cpp
# )
# ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(octomap_reconstruction_node 
  src/octomap/octomap_reconstruction_node.cpp
)

ament_target_dependencies(octomap_reconstruction_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_ros
  octomap_msgs
  tf2_ros
  message_filters
  nav_msgs
  tf2_eigen
)

target_link_libraries(octomap_reconstruction_node
  ${PCL_LIBRARIES}
  ${octomap_LIBRARIES}
)

install(TARGETS 
# mono 
# rgbd 
# stereo
# stereo-inertial
imu_rgbd
octomap_reconstruction_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch config vocabulary
 DESTINATION share/${PROJECT_NAME}/)

ament_package()

