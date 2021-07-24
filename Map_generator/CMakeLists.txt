cmake_minimum_required(VERSION 3.0.2)
project(Map_generator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# use C++11
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  tf2_ros
  nav_msgs
  message_filters
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    tf2_msgs
    pcl_ros
    nav_msgs
    message_filters
)

###########
## Build ##
###########

find_package(PCL)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ executable
add_executable(${PROJECT_NAME}
  src/Mapper.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY launch config rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

