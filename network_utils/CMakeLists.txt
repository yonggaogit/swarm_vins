cmake_minimum_required(VERSION 3.0.2)
project(network_utils)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)

# Find ZeroMQ library
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

# Declare the executable
add_executable(pose_pub_sub src/pose_pub_sub.cpp)

# Specify libraries to link executable targets against
target_link_libraries(pose_pub_sub
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARIES}
)

# Add cmake target dependencies of the executable/library
add_dependencies(pose_pub_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})