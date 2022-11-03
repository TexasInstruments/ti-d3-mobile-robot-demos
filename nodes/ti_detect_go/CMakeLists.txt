cmake_minimum_required(VERSION 3.0.2)
project(ti_detect_go)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  ti_mmwave_tracker_rospkg
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    ti_mmwave_tracker_rospkg
)

set(app ti_detect_go)
add_executable(${app}
  src/detect_go.cpp
)
target_link_libraries(${app}
  ${catkin_LIBRARIES}
)
add_dependencies(${app}
  ${ti_mmwave_tracker_rospkg_EXPORTED_TARGETS}
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
