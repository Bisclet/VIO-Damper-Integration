cmake_minimum_required(VERSION 3.8)
project(cocoro_evaluation)

# =====================
# Compiler options
# =====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Find dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


find_package(rosidl_default_generators REQUIRED)

# =====================
# Generate custom messages
# =====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float32Stamped.msg"
  "msg/Int64Stamped.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# =====================
# Executable
# =====================
add_executable(measure_intrinsics
  src/measure_intrinsics.cpp
)

ament_target_dependencies(
  measure_intrinsics
  rclcpp
  sensor_msgs
)

target_link_libraries(measure_intrinsics Eigen3::Eigen)

set_target_properties(measure_intrinsics PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

install(
  TARGETS measure_intrinsics
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(measure_metrics
  src/measure_metrics.cpp
)

ament_target_dependencies(
  measure_metrics
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
)


rosidl_target_interfaces(
  measure_metrics
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# =====================
# C++ standard
# =====================
set_target_properties(measure_metrics PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

# =====================
# Install
# =====================
install(
  TARGETS measure_metrics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
