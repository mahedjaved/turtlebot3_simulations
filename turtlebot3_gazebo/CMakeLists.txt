################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  gazebo_ros
)

################################################################################
# Declare catkin package
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf gazebo_ros
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Example executable (replace with your actual source files)
# add_executable(example_node src/example_node.cpp)
# target_link_libraries(example_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
# install(TARGETS example_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)