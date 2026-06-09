cmake_minimum_required(VERSION 3.8)
project(peak_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required for compiled dependencies
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

####################################
## External (non-ROS) dependency  ##
####################################
include(FetchContent)
FetchContent_Declare(
  peak_micropulse
  GIT_REPOSITORY https://github.com/MShields1986/peak_micropulse_driver
  GIT_TAG        feature/nodelet_compatibility
)
FetchContent_MakeAvailable(peak_micropulse)

###############
## Find deps ##
###############
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

##########################
## Generate interfaces  ##
##########################
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Ascan.msg
  msg/Observation.msg
  srv/StreamData.srv
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

###################
# Component library #
###################
add_library(${PROJECT_NAME}_components SHARED
  src/peak_component.cpp
  src/reconstruction_component.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS 2 dependencies via their exported modern CMake targets.
# (ament_target_dependencies() was removed in recent Rolling; ${<pkg>_TARGETS}
#  is the supported replacement and carries include dirs + libs transitively.)
# The external Peak hardware handler and the interfaces generated in this
# package are linked into the same target.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_components
  ${rclcpp_TARGETS}
  ${rclcpp_components_TARGETS}
  ${ament_index_cpp_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  PeakMicroPulseHandler
  ${cpp_typesupport_target}
)

# Register the composable components
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "peak_namespace::PeakComponent"
  "reconstruction_namespace::ReconstructionComponent"
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  mps
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
