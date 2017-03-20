cmake_minimum_required(VERSION 2.8.3)
project(beacon_tracker)

find_package(OpenCV 3.1.0 REQUIRED)
find_package(CUDA REQUIRED)

set (CMAKE_CXX_STANDARD 11)
set (OPENCV_CUDA_LIB_DIRS /usr/local/lib/)
set (OPENCV_CUDA_INCLUDE_DIRS /usr/local/include/opencv2/)

link_directories(${CUDA_LIBRARY_DIRS} ${OPENCV_CUDA_LIB_DIRS} )
include_directories(include ${OPENCV_CUDA_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

cuda_add_library(BeaconDetector src/gpuHelper.cu src/ColourFilterCircleDetector.cpp src/CombinedCircleDetector.cpp src/CuCombinedCircleDetector.cpp )
target_link_libraries(BeaconDetector opencv_cudaimgproc ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} )


catkin_package(

)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		geometry_msgs
		sensor_msgs
		cv_bridge
		image_transport
)

include_directories(include  ${catkin_INCLUDE_DIRS})

add_executable(beacon_tracker_node src/beacon_tracker_node.cpp)

target_link_libraries(beacon_tracker_node
	 BeaconDetector
   ${catkin_LIBRARIES}

)
