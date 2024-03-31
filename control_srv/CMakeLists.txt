cmake_minimum_required(VERSION 2.8.3)
project(control_srv)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  roslib
  geometry_msgs
  actionlib_msgs
  actionlib
)

add_compile_options(-std=c++11)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_message_files(
    FILES
    robot_state.msg
)

add_service_files(
    FILES
    start_control.srv
)
add_action_files(FILES control_task.action)

generate_messages(
    DEPENDENCIES
    actionlib
    actionlib_msgs
    std_msgs
    geometry_msgs
    
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    roscpp 
    rospy
    roslib
    actionlib_msgs
    actionlib
  DEPENDS
    Boost
)

add_executable(server src/run_server.cpp)
add_executable(client src/run_client.cpp)

target_link_libraries(
  server
  ${catkin_LIBRARIES}
)
target_link_libraries(
  client
  ${catkin_LIBRARIES}
)
