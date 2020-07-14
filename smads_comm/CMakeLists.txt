cmake_minimum_required(VERSION 3.0.2)
project(smads_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

# Message generation
add_action_files(DIRECTORY action FILES Waypoint.action)
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)


catkin_package()

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(gps_translator_node src/gps_translator.cc)
target_link_libraries(gps_translator_node ${catkin_LIBRARIES})
add_dependencies(gps_translator_node smads_comm_generate_messages_cpp)
