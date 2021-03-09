cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(painting)

# cmake build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug"
        AND NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\".
        Allowed values are Debug, Release")
endif()

set(CMAKE_TOOLCHAIN_FILE "")

include_directories(include)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.cc)

add_library(painting SHARED ${SOURCES})