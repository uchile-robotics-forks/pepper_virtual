cmake_minimum_required(VERSION 2.8.3)
project(pepper_gazebo_plugin)

find_package(catkin REQUIRED roscpp std_msgs geometry_msgs nav_msgs tf control_toolbox)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS gazebo
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf control_toolbox
    INCLUDE_DIRS include
    LIBRARIES
)

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_library(pepper_gazebo_ros_force_omni_base src/gazebo_ros_force_omni_base.cpp)
target_link_libraries(pepper_gazebo_ros_force_omni_base ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY launch worlds
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  pepper_gazebo_ros_force_omni_base
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
