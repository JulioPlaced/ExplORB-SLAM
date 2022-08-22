cmake_minimum_required(VERSION 3.0.2)
project(frontier_detector)

INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
IF(COMPILER_SUPPORTS_CXX11)
	add_compile_options(-std=c++11)
ELSE()
	MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        std_msgs
                                      	tf
                                      	geometry_msgs
                                      	nav_msgs
                                      	visualization_msgs
                                      	message_generation
                                      	dynamic_reconfigure
										octomap_ros
										orb_slam2_ros)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 4.0 QUIET)
message("Loaded OpenCV 4.0.")
if(NOT OpenCV_FOUND)
	find_package(OpenCV 3.0 QUIET)
	message("Loaded OpenCV 3.0.")
	if(NOT OpenCV_FOUND)
		find_package(OpenCV 2.4.3 QUIET)
		message("Loaded OpenCV 2.4.3.")
		if(NOT OpenCV_FOUND)
			message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
		endif()
	endif()
endif()

add_message_files(FILES PointArray.msg)

#add_service_files(
#		DIRECTORY srv
#		FILES GetOctomap.srv
#)

generate_messages(DEPENDENCIES  std_msgs
								#octomap_msgs
                                geometry_msgs)

generate_dynamic_reconfigure_options(cfg/informationGain.cfg)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS roscpp
                              rospy
                              std_msgs
                              tf
                              geometry_msgs
                              nav_msgs
                              visualization_msgs
                              dynamic_reconfigure
                              message_runtime
		                      octomap_ros)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
					${EIGEN3_INCLUDE_DIR})


# Frontier detectors
add_executable(${PROJECT_NAME}_global_rrt src/GlobalRRTDetector.cpp
		src/Functions.cpp src/Mtrand.cpp)

target_link_libraries(${PROJECT_NAME}_global_rrt	${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_local_rrt src/LocalRRTDetector.cpp
		src/Functions.cpp src/Mtrand.cpp)

target_link_libraries(${PROJECT_NAME}_local_rrt	${catkin_LIBRARIES})


# Octomapper
add_executable(${PROJECT_NAME}_octomapper	src/OctoMapper.cpp)

add_dependencies(${PROJECT_NAME}_octomapper	${catkin_EXPORTED_TARGETS}
											${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_octomapper	${catkin_LIBRARIES}
													${EIGEN3_LIBS}
													${OpenCV_LIBS})

# Gridmapper
add_executable(${PROJECT_NAME}_gridmapper	src/GridMapper.cpp)

add_dependencies(${PROJECT_NAME}_gridmapper	${catkin_EXPORTED_TARGETS}
		${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_gridmapper	${catkin_LIBRARIES}
		${EIGEN3_LIBS}
		${OpenCV_LIBS})