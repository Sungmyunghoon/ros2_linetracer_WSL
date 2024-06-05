cmake_minimum_required(VERSION 3.16.3)
project(linetracer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(/usr/local/include/dynamixel_sdk)

add_executable(psub_wsl src/psub_wsl.cpp)
ament_target_dependencies(psub_wsl rclcpp sensor_msgs OpenCV geometry_msgs)
#target_include_directories(sub PUBLIC /usr/local/include/dynamixel_sdk)


install(TARGETS
  psub_wsl
  DESTINATION lib/${PROJECT_NAME})

ament_package()