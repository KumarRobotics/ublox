cmake_minimum_required(VERSION 2.8.3)
project(ublox_gps)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  ublox_msgs
  ublox_serialization
  diagnostic_updater
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp ublox_msgs ublox_serialization)

# include boost
find_package(Boost REQUIRED COMPONENTS system regex thread)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

# include other ublox packages
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${catkin_INCLUDE_DIRS})

# link pthread
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# build library
add_library(ublox_gps src/gps.cpp)

# fix msg compile order bug
add_dependencies(ublox_gps ${catkin_EXPORTED_TARGETS})

target_link_libraries(ublox_gps
  boost_system
  boost_regex
  boost_thread
)

target_link_libraries(ublox_gps
  ${catkin_LIBRARIES}
)

# build node
add_executable(ublox_gps_node src/node.cpp src/mkgmtime.c)
set_target_properties(ublox_gps_node PROPERTIES OUTPUT_NAME ublox_gps)

target_link_libraries(ublox_gps_node boost_system boost_regex boost_thread)
target_link_libraries(ublox_gps_node ${catkin_LIBRARIES})
target_link_libraries(ublox_gps_node ublox_gps)

install(TARGETS ublox_gps ublox_gps_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
