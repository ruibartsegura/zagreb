cmake_minimum_required(VERSION 3.0.2)
project(faceDetection)

## Find OpenCV package
find_package(OpenCV REQUIRED)

## Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare the executable
add_executable(objectDetection objectDetection.cpp)

## Link the executable with OpenCV libraries
target_link_libraries(objectDetection ${OpenCV_LIBS})
