cmake_minimum_required(VERSION 3.0.2)
project(turtle_game)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs  geometry_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME}_node src/turtle_game.cpp ${src_files})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES})
  
