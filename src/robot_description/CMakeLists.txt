cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        std_msgs
                                        turtlebot3_teleop
                                        urdf
                                        rviz
                                        xacro
                                        gazebo_ros
                                        robot_state_publisher
                                        move_base
                                        octomap_rviz_plugins)

catkin_package(CATKIN_DEPENDS   roscpp
                                rospy
                                std_msgs
                                turtlebot3_teleop
                                urdf
                                rviz
                                xacro
                                gazebo_ros
                                robot_state_publisher
                                move_base
                                octomap_rviz_plugins)

include_directories(${catkin_INCLUDE_DIRS})
