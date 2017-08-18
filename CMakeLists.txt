cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME behaviour_planning)
project(${PROJECT_NAME})

set(behaviour_planning_sources main.cpp vehicle.h vehicle.cpp road.h road.cpp)

add_executable(${PROJECT_NAME} ${behaviour_planning_sources})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)


