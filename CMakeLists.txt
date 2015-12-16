cmake_minimum_required(VERSION 2.8.3)
project(msd78_vision_guided_motion)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)

cs_add_executable(robot_butler src/robot_butler.cpp)
target_link_libraries(robot_butler cwru_pcl_utils baxter_traj_streamer ${catkin_LIBRARIES})

cs_install()
cs_export()
    