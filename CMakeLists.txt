cmake_minimum_required(VERSION 2.8.3)
project(smads_core)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS regex)
find_package(Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS 
    ros_type_introspection
    topic_tools
    sensor_msgs
    geometry_msgs
    message_generation
    genmsg
    tf
    )

#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
    LIBRARIES

    CATKIN_DEPENDS
    ros_type_introspection
    topic_tools
    geometry_msgs
    message_runtime

    DEPENDS
    )

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)


add_executable(message_translator src/message_translator.cpp)
target_link_libraries(message_translator ${catkin_LIBRARIES})


