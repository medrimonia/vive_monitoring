cmake_minimum_required(VERSION 2.8)
project(vive_monitoring)

find_package(catkin REQUIRED COMPONENTS
  hl_monitoring
  vive_provider
  )

#Enable C++11
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vive_visualizer programs/vive_visualizer.cpp)
target_link_libraries(vive_visualizer ${catkin_LIBRARIES})
