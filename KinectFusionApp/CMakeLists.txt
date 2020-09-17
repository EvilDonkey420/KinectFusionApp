cmake_minimum_required(VERSION 3.0)

project(KinectFusionApp)

# Use modern C++
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

# ------------------------------------------------
#                 Dependencies
# ------------------------------------------------
## CUDA
find_package(CUDA 10.2 REQUIRED)
IF (CUDA_FOUND)
    include_directories("${CUDA_INCLUDE_DIRS}")
    # Optional: Specify the arch of your CUDA hardware here
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11 -gencode arch=compute_61,code=sm_61)
ENDIF ()

## OpenCV
# Optional: Set OpenCV_DIR if you want to use a custom version of OpenCV
SET("OpenCV_DIR" "/opt/opencv/share/OpenCV")
find_package(OpenCV 3.0 REQUIRED)
if (OpenCV_INCLUDE_DIRS)
    include_directories("${OpenCV_INCLUDE_DIRS}")
    link_directories(${OpenCV_DIR}/lib)
endif (OpenCV_INCLUDE_DIRS)

## Eigen3
find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif (EIGEN3_INCLUDE_DIR)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set(OPENNI2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/OpenNI2/Include" CACHE PATH "OpenNI2 Include directory")
set(OPENNI2_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/OpenNI2/Bin/x64-Release/libOpenNI2.so" CACHE PATH "OpenNI2 library")

# OpenNI2
find_package(OpenNI2 REQUIRED)
if (OPENNI2_FOUND)
    include_directories("${OPENNI2_INCLUDE_DIR}")
endif (OPENNI2_FOUND)

## Intel RealSense
find_package(realsense2 REQUIRED)
if (realsense2_FOUND)
    include_directories("${realsense2_INCLUDE_DIR}")
endif (realsense2_FOUND)
# ------------------------------------------------

include_directories(${PROJECT_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/KinectFusionLib/include)

file(GLOB KinectFusionApp_SRCS ${PROJECT_SOURCE_DIR}/*.cpp)

cuda_add_executable(KinectFusionApp ${KinectFusionApp_SRCS})
target_link_libraries(KinectFusionApp ${OpenCV_LIBS} ${OPENNI2_LIBRARY} ${realsense2_LIBRARY} KinectFusion)
