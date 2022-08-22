cmake_minimum_required(VERSION 3.0.2)
project(kobuki_plugins)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS gazebo_ros
                                        gazebo_plugins
                                        geometry_msgs
                                        kobuki_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES gazebo_ros_kobuki
               CATKIN_DEPENDS gazebo_ros
                              gazebo_plugins
                              geometry_msgs
                              kobuki_msgs
                              nav_msgs
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 ")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_ros_kobuki src/gazebo_ros_kobuki.cpp
                              src/gazebo_ros_kobuki_updates.cpp
                              src/gazebo_ros_kobuki_loads.cpp)
add_dependencies(gazebo_ros_kobuki ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_kobuki
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ros_kobuki
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
