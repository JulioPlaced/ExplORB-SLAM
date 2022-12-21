cmake_minimum_required(VERSION 3.0.2)
project(decision_maker)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        std_msgs
                                        orb_slam2_ros)

# catkin_package(INCLUDE_DIRS     include
catkin_package(CATKIN_DEPENDS   roscpp
                                rospy
                                std_msgs)

# include_directories(include ${catkin_INCLUDE_DIRS})
