cmake_minimum_required(VERSION 3.8)
project(zed_minimal_wrapper)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
  message(FATAL_ERROR "ZED static libraries not available on Windows")
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-format-extra-args)
endif()

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ZED + OpenCV + CUDA
find_package(ZED 3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

# OS libs (Linux)
if(NOT WIN32)
  set(SPECIAL_OS_LIBS pthread X11)
endif()

# ===============================
# Single executable
# ===============================
add_executable(zed2i src/zed2i.cpp)

target_include_directories(zed2i PRIVATE
  ${ZED_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

if (LINK_SHARED_ZED)
  set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
  set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

target_link_libraries(zed2i
  ${ZED_LIBS}
  ${OpenCV_LIBRARIES}
  ${SPECIAL_OS_LIBS}
)

ament_target_dependencies(zed2i rclcpp nav_msgs sensor_msgs cv_bridge)

target_compile_features(zed2i PUBLIC cxx_std_17)

# Install
install(TARGETS zed2i
  DESTINATION lib/${PROJECT_NAME})

ament_package()
