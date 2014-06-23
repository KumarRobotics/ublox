cmake_minimum_required(VERSION 2.8.3)
project(ublox_gps)
find_package(catkin REQUIRED roscpp ublox_msgs ublox_serialization) #sensor_msgs

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp ublox_msgs ublox_serialization) #sensor_msgs

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# include boost
find_package(Boost REQUIRED COMPONENTS system regex thread)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

# include other ublox packages
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${ublox_msgs_INCLUDE_DIRS})
include_directories(${ublox_serialization_INCLUDE_DIRS})

# link pthread
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -lrt -pthread")

# build library
add_library(ublox_gps src/gps.cpp)

# fix msg compile order bug
add_dependencies(ublox_gps ${ublox_msgs_EXPORTED_TARGETS})

target_link_libraries(ublox_gps boost_system boost_regex boost_thread)
target_link_libraries(ublox_gps ${roscpp_LIBRARIES} ${ublox_msgs_LIBRARIES})

# build node
add_executable(ublox_gps_node src/node.cpp)
set_target_properties(ublox_gps_node PROPERTIES OUTPUT_NAME ublox_gps)

target_link_libraries(ublox_gps_node boost_system boost_regex boost_thread)
target_link_libraries(ublox_gps_node ${roscpp_LIBRARIES} ${ublox_msgs_LIBRARIES})
target_link_libraries(ublox_gps_node ublox_gps)
