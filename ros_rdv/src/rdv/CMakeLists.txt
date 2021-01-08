cmake_minimum_required(VERSION 3.1.3)
project(indy7_robotiq_moveit_interface)

find_package(catkin REQUIRED
  COMPONENTS
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
    roscpp
    std_msgs
)

catkin_package(
  LIBRARIES indy7_robotiq_moveit_interface
  CATKIN_DEPENDS std_msgs roscpp
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(picknplace src/pick.cpp)
add_dependencies(picknplace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(picknplace ${catkin_LIBRARIES})


add_executable(rdv_picknplace src/rdv.cpp)
add_dependencies(rdv_picknplace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rdv_picknplace ${catkin_LIBRARIES})
