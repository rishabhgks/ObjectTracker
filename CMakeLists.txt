project(ObjectTracker)
cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


find_package(OpenCV 4.1  COMPONENTS core imgproc highgui imgcodecs tracking REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(ObjectTracker main.cpp)

target_link_libraries(ObjectTracker ${OpenCV_LIBS})

add_executable(DoubleObjectTracker double_detect.cpp)

target_link_libraries(DoubleObjectTracker ${OpenCV_LIBS})