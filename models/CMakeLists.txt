cmake_minimum_required(VERSION 3.5)
project(lab_gazebo_models)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(hinge_control SHARED src/hinge_control.cpp)
target_link_libraries(
  hinge_control
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(hinge_control rclcpp std_msgs)

add_library(link_state_publisher SHARED src/link_state_publisher.cpp)
target_link_libraries(
  link_state_publisher
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(
  link_state_publisher rclcpp std_msgs geometry_msgs gazebo_msgs)

install(TARGETS hinge_control
  LIBRARY DESTINATION lib)
install(TARGETS link_state_publisher
  LIBRARY DESTINATION lib)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
