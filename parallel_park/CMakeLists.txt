cmake_minimum_required(VERSION 2.8.3)
project(parallel_park)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#
find_package(catkin REQUIRED COMPONENTS
     roscpp
     tf2_ros
     geometry_msgs
     std_msgs
     message_generation
)

add_message_files(
   FILES
   ServoCtrlMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   DEPENDS system_lib
)

include_directories(include msg ${catkin_INCLUDE_DIRS})

add_executable(parallel_park parallel_park.cpp)
target_link_libraries(parallel_park ${catkin_LIBRARIES})
