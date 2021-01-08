cmake_minimum_required(VERSION 3.0.2)
project(ros_rdv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  serial
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
  intfloatstring.srv
  rdv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  #INCLUDE_DIRS include -> 아마 h 파일 모아 놓는데가 아닐까 싶음.
  LIBRARIES ros_rdv
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server
	${catkin_LIBRARIES}
)
add_dependencies(add_two_ints_server ${PROJECT_NAME}_generate_messages_cpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client
	${catkin_LIBRARIES}
)
add_dependencies(add_two_ints_client ${PROJECT_NAME}_generate_messages_cpp)

add_executable(intfloatstring_client src/intfloatstring_client.cpp)
target_link_libraries(intfloatstring_client
	${catkin_LIBRARIES}
)
add_dependencies(intfloatstring_client ${PROJECT_NAME}_generate_messages_cpp)

add_executable(intfloatstring_server src/intfloatstring_server.cpp)
target_link_libraries(intfloatstring_server
	${catkin_LIBRARIES}
)
add_dependencies(intfloatstring_server ${PROJECT_NAME}_generate_messages_cpp)


add_executable(rdv_server src/rdv_server.cpp)
target_link_libraries(rdv_server
  ${catkin_LIBRARIES}
)
add_dependencies(rdv_server ${PROJECT_NAME}_generate_messages_cpp)

add_executable(rdv_client src/rdv_client.cpp)
target_link_libraries(rdv_client
	${catkin_LIBRARIES}
)
add_dependencies(rdv_client ${PROJECT_NAME}_generate_messages_cpp)
