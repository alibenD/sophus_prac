#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: CMakeLists.txt
# @author: aliben.develop@gmail.com
# @create_date: 2018-08-16 09:23:44
# @last_modified_date: 2018-08-17 12:15:56
# @description: TODO
#---****************************************************************---

# Cmake version required
cmake_minimum_required(VERSION 2.8)

#project name
project(test_sophus)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-g -std=c++14 -march=native -O0")

# Set Dependence PATH
  # Example: set ( PACKAGE_DIR PATH )
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/)

# Find *Config.cmake
  # Example: find_package(OpenCV REQUIRED)
  FIND_PACKAGE(Sophus REQUIRED)
  FIND_PACKAGE(Eigen3 REQUIRED)
  INCLUDE_DIRECTORIES(${SOPHUS_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
  MESSAGE( STATUS "Sophus Path: ${SOPHUS_INCLUDE_DIRS}" )
  MESSAGE( STATUS "Eigen Path: ${EIGEN3_INCLUDE_DIRS}")

  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
  ADD_SUBDIRECTORY(test)
# EXECUTABLE
  # Example: add_executable( EXEC_NAME SRC_FILE_NAME_LIST )

# TARGET LINK
  # Example: target_link_libraries( EXEC_NAME LIBPATH ) ...LIB_PATH e.g. ${OPENCV_LIBS}


