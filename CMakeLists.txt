cmake_minimum_required(VERSION 2.8.3)
project(led_driver_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
)


include_directories(include/led_driver_controller)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver
    src/main.cpp
    src/led_driver_controller.cpp
    src/driver.cpp

    include/led_driver_controller/led_driver_controller.h
    include/led_driver_controller/driver.h
)

target_link_libraries(driver ${catkin_LIBRARIES})
add_definitions(-std=c++11)
