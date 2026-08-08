cmake_minimum_required(VERSION 3.8)
project(retail_nav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavStatus.msg"
  "msg/NavPose.msg"
  "msg/NavEvent.msg"
  "msg/StationInfo.msg"
  "msg/HealthSnapshot.msg"
  "srv/Init.srv"
  "srv/GetHealth.srv"
  "srv/ClearError.srv"
  "srv/StartMapping.srv"
  "srv/CancelMapping.srv"
  "srv/LoadMap.srv"
  "srv/Relocate.srv"
  "srv/ListStations.srv"
  "srv/SyncStations.srv"
  "srv/ExportMapPackage.srv"
  "srv/ExportVendorMapPackage.srv"
  "srv/CancelNav.srv"
  "srv/Pause.srv"
  "srv/Resume.srv"
  "srv/Hold.srv"
  "srv/ReleaseHold.srv"
  "action/StopMapping.action"
  "action/GoToStation.action"
  "action/GoToPose.action"
  "action/MoveRelative.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
