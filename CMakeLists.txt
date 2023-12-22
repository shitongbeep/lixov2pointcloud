cmake_minimum_required(VERSION 3.0.2)
project(livox_convert)

add_compile_options(-std=c++17)
SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

find_package(PCL 1.10 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  std_msgs
	sensor_msgs
	message_generation
)

add_message_files(FILES
	CustomPoint.msg
	CustomMsg.msg
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
  DEPENDS EIGEN PCL
  INCLUDE_DIRS
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	include
)

add_executable(convert src/convert.cpp)
target_link_libraries(convert ${catkin_LIBRARIES} ${PCL_LIBRARIES})