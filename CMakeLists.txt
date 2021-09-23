cmake_minimum_required(VERSION 3.0.2)
project(prophesee_tool)
set(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)     # c++14 is required by prophsee's raw-to-csv

find_package(catkin REQUIRED COMPONENTS
    prophesee_event_msgs
    roscpp
    std_msgs
    cv_bridge
    image_transport
    rosbag
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED) 
find_package(MetavisionSDK COMPONENTS driver REQUIRED)
find_package(MetavisionSDK COMPONENTS core REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/celex_ros.cpp
# )

link_directories()

add_executable(bag2txt src/bag2txt.cpp)
target_link_libraries(bag2txt ${catkin_LIBRARIES} ${OpenCV_LIBS} gflags MetavisionSDK::driver)


add_executable(raw2csv src/raw2csv.cpp)
target_link_libraries(raw2csv ${catkin_LIBRARIES} ${OpenCV_LIBS} MetavisionSDK::core MetavisionSDK::driver Boost::program_options)
