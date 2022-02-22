cmake_minimum_required(VERSION 3.0.2)
project(ublox_msg_filters)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ublox_msgs
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ublox_msgs message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_example
  src/example.cpp
)
add_dependencies(${PROJECT_NAME}_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_example
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_example
  PROPERTIES OUTPUT_NAME example PREFIX ""
)

add_compile_options(
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-unused-function
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
