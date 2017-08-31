cmake_minimum_required(VERSION 3.8)
project(prog_agent)

include_directories(/opt/ros/kinetic/include)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp prob_agent.h)
add_executable(prog_agent ${SOURCE_FILES})