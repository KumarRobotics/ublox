cmake_minimum_required(VERSION 2.8.3)
project(ublox_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation ublox_serialization std_msgs sensor_msgs)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime ublox_serialization std_msgs sensor_msgs)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${ublox_serialization_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/ublox_msgs.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
link_libraries(${PROJECT_NAME} ${message_runtime_LIBRARIES})
