cmake_minimum_required(VERSION 2.8.3)
project(marker_tracker)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	image_transport
	cv_bridge
	camera_info_manager
)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS 
	  include
	LIBRARIES 
	  ${PROJECT_NAME}
	CATKIN_DEPENDS
	  cmake_modules
	  roscpp
	  image_transport
	  cv_bridge
	  camera_info_manager
)

add_custom_target(${PROJECT_NAME}_HEADERS
	SOURCES
 	include/marker_tracker_node.hpp
)

include_directories(
	include
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} src/marker_tracker_node.cpp)

add_library(MarkerTracker include/MarkerTracker.hpp src/MarkerTracker.cpp)
target_link_libraries(MarkerTracker ${OpenCV_LIBRARIES})

add_library(Kalman3D include/Kalman3D.hpp src/Kalman3D.cpp)
target_link_libraries(Kalman3D ${OpenCV_LIBRARIES})


add_executable(marker_tracker_node src/marker_tracker_node.cpp)
target_link_libraries(marker_tracker_node MarkerTracker Kalman3D
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)


