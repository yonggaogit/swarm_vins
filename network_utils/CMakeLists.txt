cmake_minimum_required(VERSION 3.0.2)
project(network_utils)

add_compile_options(-std=c++11)
# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

# Find ZeroMQ library
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

find_package(Boost REQUIRED COMPONENTS system thread serialization)
find_package(Protobuf REQUIRED)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

# 生成Protobuf文件
set(PROTO_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/messages.proto
)

# 生成头文件和源文件
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs
  DEPENDS Boost
)

# Declare the executable
add_executable(pose_pub_sub src/pose_pub_sub.cpp)

# Specify libraries to link executable targets against
target_link_libraries(pose_pub_sub
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARIES}
)

add_executable(drone_node src/drone_node.cpp ${PROTO_SRCS})
target_link_libraries(drone_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(server_node src/server_node.cpp ${PROTO_SRCS})
target_link_libraries(server_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

# Add cmake target dependencies of the executable/library
add_dependencies(pose_pub_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drone_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})