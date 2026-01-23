cmake_minimum_required(VERSION 3.8)
project(next2_msgs)

# ใช้ C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)


# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExampleAction.action"
  "action/JackingOperation.action"
  "action/JackDaniel.action"
  "action/SerialOperation.action"

  "msg/ExampleMessage.msg"
  "msg/CanOpenStatus.msg"
  "msg/CanOpenTwist.msg"
  "msg/JackState.msg"
  "msg/JackInterface.msg"
  "msg/MotorSpin.msg"
  "msg/RobotMode.msg"
  "msg/Diagnostics.msg"
  "msg/DiagnosticsModern.msg"
  "msg/Devices.msg"
  "msg/Error.msg"
  "msg/SeerIO.msg"
  "msg/SeerRobotIOStatus.msg"
  "msg/LEDControl.msg"
  "msg/LEDState.msg"

  "srv/ExampleService.srv"
  "srv/ActionController.srv"
  "srv/SetIOs.srv"

  DEPENDENCIES std_msgs diagnostic_msgs action_msgs

)

# ament package
ament_package()
