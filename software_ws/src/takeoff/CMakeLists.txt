cmake_minimum_required(VERSION 3.5)
project(takeoff)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})


find_package(geodetic_utils REQUIRED)
include_directories(${geodetic_utils_INCLUDE_DIRS})
