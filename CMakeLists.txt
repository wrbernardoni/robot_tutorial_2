cmake_minimum_required(VERSION 2.8.3)
project(robot_tutorial_1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
find_package(gazebo REQUIRED)

# Declare dependencies
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_tutorial_1
#  CATKIN_DEPENDS gazebo_ros roscpp
#  DEPENDS system_libi
  DEPENDS
    roscpp
    gazebo_ros
    message_runtime
)

# Link to gazebo
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Include dirs.
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Compile plugin
add_library(wb_plugin plugins/wb_plugin.cc)
target_link_libraries(wb_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

# Compile keyin
add_executable(keyin src/keyin.cc)
target_link_libraries(keyin ${catkin_LIBRARIES})
add_dependencies(keyin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


