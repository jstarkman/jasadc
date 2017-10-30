cmake_minimum_required(VERSION 3.5)
project(jasadc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")

set(target main)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target}
  "rclcpp"
  "std_msgs")
install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
