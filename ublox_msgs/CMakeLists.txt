cmake_minimum_required(VERSION 3.5)
project(ublox_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ublox_serialization REQUIRED)

set(msg_files
  "msg/Ack.msg"
  "msg/AidALM.msg"
  "msg/AidEPH.msg"
  "msg/AidHUI.msg"
  "msg/CfgANT.msg"
  "msg/CfgCFG.msg"
  "msg/CfgDAT.msg"
  "msg/CfgDGNSS.msg"
  "msg/CfgGNSSBlock.msg"
  "msg/CfgGNSS.msg"
  "msg/CfgHNR.msg"
  "msg/CfgINFBlock.msg"
  "msg/CfgINF.msg"
  "msg/CfgMSG.msg"
  "msg/CfgNAV5.msg"
  "msg/CfgNAVX5.msg"
  "msg/CfgNMEA6.msg"
  "msg/CfgNMEA7.msg"
  "msg/CfgNMEA.msg"
  "msg/CfgPRT.msg"
  "msg/CfgRATE.msg"
  "msg/CfgRST.msg"
  "msg/CfgSBAS.msg"
  "msg/CfgTMODE3.msg"
  "msg/CfgUSB.msg"
  "msg/EsfINS.msg"
  "msg/EsfMEAS.msg"
  "msg/EsfRAWBlock.msg"
  "msg/EsfRAW.msg"
  "msg/EsfSTATUS.msg"
  "msg/EsfSTATUSSens.msg"
  "msg/HnrPVT.msg"
  "msg/Inf.msg"
  "msg/MgaGAL.msg"
  "msg/MonGNSS.msg"
  "msg/MonHW6.msg"
  "msg/MonHW.msg"
  "msg/MonVERExtension.msg"
  "msg/MonVER.msg"
  "msg/NavATT.msg"
  "msg/NavCLOCK.msg"
  "msg/NavDGPS.msg"
  "msg/NavDGPSSV.msg"
  "msg/NavDOP.msg"
  "msg/NavPOSECEF.msg"
  "msg/NavPOSLLH.msg"
  "msg/NavPVT7.msg"
  "msg/NavPVT.msg"
  "msg/NavRELPOSNED9.msg"
  "msg/NavRELPOSNED.msg"
  "msg/NavSAT.msg"
  "msg/NavSATSV.msg"
  "msg/NavSBAS.msg"
  "msg/NavSBASSV.msg"
  "msg/NavSOL.msg"
  "msg/NavSTATUS.msg"
  "msg/NavSVINFO.msg"
  "msg/NavSVINFOSV.msg"
  "msg/NavSVIN.msg"
  "msg/NavTIMEGPS.msg"
  "msg/NavTIMEUTC.msg"
  "msg/NavVELECEF.msg"
  "msg/NavVELNED.msg"
  "msg/RxmALM.msg"
  "msg/RxmEPH.msg"
  "msg/RxmRAW.msg"
  "msg/RxmRAWSV.msg"
  "msg/RxmRAWXMeas.msg"
  "msg/RxmRAWX.msg"
  "msg/RxmRTCM.msg"
  "msg/RxmSFRB.msg"
  "msg/RxmSFRBX.msg"
  "msg/RxmSVSI.msg"
  "msg/RxmSVSISV.msg"
  "msg/TimTM2.msg"
  "msg/UpdSOSAck.msg"
  "msg/UpdSOS.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
if(cpp_typesupport_target)
  add_library(${PROJECT_NAME}_lib src/ublox_msgs.cpp)
  target_include_directories(${PROJECT_NAME}_lib PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${PROJECT_NAME}_lib
    ${cpp_typesupport_target}
    ublox_serialization::ublox_serialization
  )

  install(TARGETS ${PROJECT_NAME}_lib EXPORT ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/
    DESTINATION "include/${PROJECT_NAME}"
  )

  ament_export_include_directories("include/${PROJECT_NAME}")
  ament_export_libraries(${PROJECT_NAME}_lib)
  ament_export_targets(${PROJECT_NAME}_lib)
endif()

ament_export_dependencies(rosidl_default_runtime sensor_msgs std_msgs ublox_serialization)

ament_package()
