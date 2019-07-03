cmake_minimum_required(VERSION 2.8.3)
project(pose_to_udp)



#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    geometry_msgs
    rosconsole
    roscpp
    rostime
    tf2
    tf2_geometry_msgs
    tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})

#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        pose_to_udp
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(pose_to_udp
    src/pose_to_udp.cpp
)



#==================
# executables
#==================

add_executable(pose_to_udp_node
    src/pose_to_udp_node.cpp
)


#==================
# library link
#==================

target_link_libraries(pose_to_udp
    ${catkin_LIBRARIES}
)

target_link_libraries(pose_to_udp_node
    pose_to_udp
    ${catkin_LIBRARIES}
)
