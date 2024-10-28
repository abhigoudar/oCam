cmake_minimum_required(VERSION 3.5)
project(ocam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
# generate_dynamic_reconfigure_options(
#   cfg/cam.cfg
# )

set(DEPENDENCIES
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)

include_directories(
    include
)


add_executable(ocam src/oCam.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)
ament_target_dependencies(ocam ${DEPENDENCIES})
target_link_libraries(ocam ${OpenCV_LIBS} -lv4l2 -ludev)
# add_dependencies(ocam ${PROJECT_NAME}_gencfg)


install(
  TARGETS 
  ocam
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()