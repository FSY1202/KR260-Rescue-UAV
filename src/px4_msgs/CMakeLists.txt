cmake_minimum_required(VERSION 3.5)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ##############################################################################
# Generate ROS messages, ROS2 interfaces and IDL files #
# ##############################################################################

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
option(KR260_MINIMAL_MSGS "Build odometry and time synchronization diagnostics only" OFF)
option(KR260_MISSION_MSGS "Include one-shot Offboard control/telemetry in the minimal KR260 build" OFF)
if(KR260_MINIMAL_MSGS)
  set(PX4_MSGS "msg/VehicleOdometry.msg" "msg/TimesyncStatus.msg")
  if(KR260_MISSION_MSGS)
    list(APPEND PX4_MSGS
      "msg/VehicleLocalPosition.msg" "msg/VehicleStatus.msg"
      "msg/VehicleLandDetected.msg" "msg/FailsafeFlags.msg"
      "msg/OffboardControlMode.msg" "msg/TrajectorySetpoint.msg"
      "msg/VehicleCommand.msg" "msg/VehicleCommandAck.msg")
  endif()
else()
  file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")
endif()

# get all srv files
set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
if(KR260_MINIMAL_MSGS)
  set(PX4_SRVS)
else()
  file(GLOB PX4_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")
endif()

# Generate introspection typesupport for C and C++ and IDL files
rosidl_generate_interfaces(${PROJECT_NAME}
	${PX4_MSGS}
	${PX4_SRVS}
	DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
