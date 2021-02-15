cmake_minimum_required(VERSION 3.0.2)
project(ros_rdv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  actionlib_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
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
  rdv_py.srv
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
  std_msgs
  # actionlib_msgs 
   # Or other packages containing msgs
)

catkin_package(
  # INCLUDE_DIRS include #(헤더파일을 지정해놓은 곳)
  LIBRARIES ros_rdv
  CATKIN_DEPENDS roscpp rospy message_runtime # actionlib_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS src/rdv_py_client.py src/rdv_py_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  rdv_cup
  src/rdv_cup/rdv_run.cpp
  src/rdv_cup/rdv_core.cpp
)

target_link_libraries(rdv_cup ${catkin_LIBRARIES})
add_dependencies(rdv_cup ${catkin_EXPORTED_TARGETS})

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


# add_executable(add_two_ints_server src/service_practice/add_two_ints_server.cpp)
# target_link_libraries(add_two_ints_server
# 	${catkin_LIBRARIES}
# )
# add_dependencies(add_two_ints_server ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(add_two_ints_client src/service_practice/add_two_ints_client.cpp)
# target_link_libraries(add_two_ints_client
# 	${catkin_LIBRARIES}
# )
# add_dependencies(add_two_ints_client ${PROJECT_NAME}_generate_messages_cpp)

# # service test
# add_executable(intfloatstring_client src/service_practice/intfloatstring_client.cpp)
# target_link_libraries(intfloatstring_client
# 	${catkin_LIBRARIES}
# )
# add_dependencies(intfloatstring_client ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(intfloatstring_server src/service_practice/intfloatstring_server.cpp)
# target_link_libraries(intfloatstring_server
# 	${catkin_LIBRARIES}
# )
# add_dependencies(intfloatstring_server ${PROJECT_NAME}_generate_messages_cpp)

# # picknplace test
# add_executable(picknplace src/pick.cpp)
# add_dependencies(picknplace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(picknplace ${catkin_LIBRARIES})


