cmake_minimum_required(VERSION 3.5)
project(ublox_gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(ament_cmake_ros REQUIRED)
find_package(asio REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(ublox_serialization REQUIRED)

# build node
add_library(ublox_gps
  src/adr_udr_product.cpp
  src/gnss.cpp
  src/gps.cpp
  src/hp_pos_rec_product.cpp
  src/hpg_ref_product.cpp
  src/hpg_rov_product.cpp
  src/mkgmtime.c
  src/node.cpp
  src/raw_data_pa.cpp
  src/raw_data_product.cpp
  src/tim_product.cpp
  src/ublox_firmware.cpp
  src/ublox_firmware6.cpp
  src/ublox_firmware7.cpp
  src/ublox_firmware8.cpp
  src/ublox_firmware9.cpp)
target_include_directories(ublox_gps PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${diagnostic_updater_INCLUDE_DIRS}
)
target_link_libraries(ublox_gps PUBLIC
  ${asio_LIBRARIES}
  ${diagnostic_updater_LIBRARIES}
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  ${ublox_msgs_TARGETS}
  ublox_serialization::ublox_serialization
)

install(TARGETS ublox_gps EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(ublox_gps_node src/node_main.cpp)
target_link_libraries(ublox_gps_node PRIVATE
  rclcpp::rclcpp
  ublox_gps
)

# build logger node
add_executable(ublox_logger src/logger_node_pa.cpp src/raw_data_pa.cpp)
target_link_libraries(ublox_logger PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ublox_gps
)

install(TARGETS
  ublox_gps_node ublox_logger
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(ublox_gps
  "ublox_node::UbloxNode")

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})

ament_package()
