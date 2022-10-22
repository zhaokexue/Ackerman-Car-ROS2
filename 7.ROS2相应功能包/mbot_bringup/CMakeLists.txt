cmake_minimum_required(VERSION 3.8)
project(mbot_bringup)

add_compile_options(-fno-stack-protector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MBOT_INCLUDE_PATH "./")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  ${MBOT_INCLUDE_PATH}/include/mbot_bringup/
)

add_executable(mbot_bringup 
               src/mbot_bringup.cpp
               src/mbot.cpp
	             src/mbot_linux_serial.cpp)
ament_target_dependencies(mbot_bringup rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros nav_msgs)

install(TARGETS
  mbot_bringup
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
