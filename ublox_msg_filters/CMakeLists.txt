cmake_minimum_required(VERSION 3.5)
project(ublox_msg_filters)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(ublox_msgs REQUIRED)

set(dependencies "rclcpp" "message_filters" "ublox_msgs")

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker ${dependencies})
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener ${dependencies})
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
  ament_target_dependencies(${PROJECT_NAME}-test_exact_time_policy ${dependencies})
  target_include_directories(${PROJECT_NAME}-test_exact_time_policy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_export_include_directories(include)
ament_package()
