cmake_minimum_required(VERSION 3.10.2)
project(waypoint_editor_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  std_srvs
  tf
  message_generation
)

add_service_files(
  FILES
  EditPoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_python()
  roslint_add_test()
endif()
