cmake_minimum_required(VERSION 3.16)
project(ruka3_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
# Compile the firmware's libcanard C core and use its generated DSDL headers.
set(CYPHAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Boards/g474/Drivers/libcxxcanard")
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ruka3_system.cpp "${CYPHAL_DIR}/libs/libcanard/canard.c")
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rcl_interfaces
  diagnostic_msgs
  std_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_include_directories(${PROJECT_NAME} PRIVATE "${CYPHAL_DIR}/libs" "${CYPHAL_DIR}/libs/libcanard")

pluginlib_export_plugin_description_file(hardware_interface ruka3_control.xml)

install(
  DIRECTORY config launch ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/recovery_supervisor.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES ruka3_control.xml DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_dependencies(
  rcl_interfaces diagnostic_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  ament_target_dependencies(test_plugin_load hardware_interface pluginlib rclcpp)
  ament_add_gtest(test_limit_recovery test/test_limit_recovery.cpp)
  target_include_directories(test_limit_recovery PRIVATE include)
  add_executable(ruka3_virtual_joints test/virtual_joints.cpp "${CYPHAL_DIR}/libs/libcanard/canard.c")
  target_include_directories(ruka3_virtual_joints PRIVATE "${CYPHAL_DIR}/libs" "${CYPHAL_DIR}/libs/libcanard")
endif()

ament_package()
