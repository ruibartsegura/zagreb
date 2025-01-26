cmake_minimum_required(VERSION 3.0.2)
project(node_programming)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  turtlesim
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include

)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node 
  src/turtle_lawnmover.cpp
  src/main.cpp
)

## Link the executable to the required libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
