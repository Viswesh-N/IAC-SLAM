cmake_minimum_required(VERSION 3.10.2)
project(INDY-Slam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED 
			COMPONENTS roscpp nav_msgs
)

catkin_package(
	INCLUDE_DIRS include
	# LIBRARIES
	CATKIN_DEPENDS roscpp nav_msgs
	# DEPENDS
)

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
src/SLAM.cpp
src/noise_generator_node.cpp )


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
