cmake_minimum_required(VERSION 3.8)
project(robotdata)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/IMU.msg"
  "msg/RcChannel.msg"
  "msg/RcData.msg"
  "msg/RobotState.msg"
  "msg/RobotCmd.msg"
)

install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include
)

ament_package()
