cmake_minimum_required(VERSION 3.8)
project(prog_agent)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        CMakeLists.txt
        main.cpp
        prob_agent.h functions.h)

add_executable(prog_agent ${SOURCE_FILES})
include_directories(/opt/ros/indigo/include)