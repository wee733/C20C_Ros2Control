cmake_minimum_required(VERSION 3.16)
project(c20d_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalState.msg"
  "msg/RelativeAngleCommand.msg"
  "msg/AttitudeCommand.msg"
  "msg/AngularRateCommand.msg"
  "msg/ProportionalRateCommand.msg"
  "msg/AngleState.msg"
  "msg/AngularRateState.msg"
  "msg/CarrierData.msg"
  "msg/CameraData.msg"
  "srv/DeviceCommand.srv"
  DEPENDENCIES std_msgs
)

add_library(c20d_protocol src/protocol.cpp src/serial_port.cpp)
target_include_directories(c20d_protocol PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
add_executable(c20d_node src/driver_node.cpp)
ament_target_dependencies(c20d_node rclcpp rcl_interfaces std_msgs sensor_msgs diagnostic_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(c20d_node c20d_protocol "${cpp_typesupport_target}")
add_executable(c20d_probe src/probe.cpp)
target_link_libraries(c20d_probe c20d_protocol)

install(TARGETS c20d_protocol EXPORT export_c20d_protocol
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS c20d_node c20d_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_c20d_protocol HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol c20d_protocol)
  ament_add_gtest(test_serial test/test_serial.cpp)
  target_link_libraries(test_serial c20d_protocol util)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ros_pty test/test_ros_pty.py
    TIMEOUT 40
    APPEND_ENV
      "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    ENV
      "C20D_TEST_EXECUTABLE=$<TARGET_FILE:c20d_node>"
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
