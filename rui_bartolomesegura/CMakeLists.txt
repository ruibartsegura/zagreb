cmake_minimum_required(VERSION 3.0.2)
project(rui_bartolomesegura)

## Encontrar catkin macros y bibliotecas
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Declare catkin pkg
catkin_package()

## Eigen3 dir include
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

## Declare the executable
add_executable(${PROJECT_NAME} src/rui_bartolomesegura.cpp)

## Link the executable with the libraries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
