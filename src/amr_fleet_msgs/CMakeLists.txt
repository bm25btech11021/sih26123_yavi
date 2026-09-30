cmake_minimum_required(VERSION 3.8)
project(amr_fleet_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/TaskBid.msg"
  "msg/TaskAssignment.msg"
  "msg/BlockedResource.msg"
  "msg/FleetMetrics.msg"
  "msg/TaskDefinition.msg"
  "msg/TaskEvent.msg"
  "msg/TaskList.msg"
  "msg/CBBABid.msg"
  "msg/RobotBundle.msg"
  "msg/PlanningRequest.msg"
  "msg/PlanningResponse.msg"
  "msg/RollingHorizonPlan.msg"
  "msg/SpaceTimeReservation.msg"
  "msg/ConflictReport.msg"
  "msg/DeadlockEvent.msg"
  "msg/CoordinationStatus.msg"
  "msg/CommunicationProfile.msg"
  "msg/CommunicationMetrics.msg"
  "msg/ComputeModeEvent.msg"
  "msg/AisleBlockageEvent.msg"
  "msg/RobotHealth.msg"
  "srv/CreateTask.srv"
  "srv/ControlTask.srv"
  "srv/InjectFault.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

