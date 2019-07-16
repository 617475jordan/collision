cmake_minimum_required(VERSION 3.7)
project(collision)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp collisionDetected.cpp collisionDetected.h)
add_executable(collision ${SOURCE_FILES})