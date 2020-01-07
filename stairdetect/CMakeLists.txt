cmake_minimum_required(VERSION 2.8.3)
project(stairdetect)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  genmsg
  tf
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/StairDetect.cfg
)

# Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

# Generate services in the 'srv' folder
#add_service_files(
#   FILES
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
#   sensor_msgs
#   std_msgs
#   geometry_msgs
#   trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime sensor_msgs dynamic_reconfigure
  DEPENDS PCL OpenCV Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
# TODO: Check names of system library include directories (PCL, OpenCV)
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
 ${OpenCV_LIBRARIES}
 ${Boost_LIBRARIES} 

)

target_link_libraries(${PROJECT_NAME}_node
 ${PROJECT_NAME}
)
