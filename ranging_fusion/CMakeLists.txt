cmake_minimum_required(VERSION 2.8.3)
project(ranging_fusion)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  nlink_parser
)

find_package(Ceres REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


catkin_package()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ranging_fusion_node
	src/globalOptNode.cpp
	src/parameters.cpp
	src/globalOpt.cpp)

# add_executable(test_line test/line.cpp)
# add_executable(test_line_error test/line_error.cpp)
# add_executable(test_stable test/stable.cpp)
# add_executable(test_stable_error test/stable_error.cpp)
# add_executable(uwb_locate test/uwb_ceres_locate.cpp)
target_link_libraries(ranging_fusion_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

# target_link_libraries(test_line ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(uwb_locate ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(test_line_error ${catkin_LIBRARIES})
# target_link_libraries(test_stable ${catkin_LIBRARIES})
# target_link_libraries(test_stable_error ${catkin_LIBRARIES})