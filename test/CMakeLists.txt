#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: CMakeLists.txt
# @author: aliben.develop@gmail.com
# @create_date: 2018-08-16 09:28:53
# @last_modified_date: 2018-08-17 14:17:51
# @description: TODO
#---****************************************************************---

# Cmake version required

#project name

# Set Dependence PATH
  # Example: set ( PACKAGE_DIR PATH )

# Find *Config.cmake
  # Example: find_package(OpenCV REQUIRED)

# EXECUTABLE
  # Example: add_executable( EXEC_NAME SRC_FILE_NAME_LIST )
  ADD_EXECUTABLE(test_so3 test_so3.cpp)
  ADD_EXECUTABLE(test_se3 test_se3.cpp)
  ADD_EXECUTABLE(test_eigen test_eigen.cpp)

# TARGET LINK
  # Example: target_link_libraries( EXEC_NAME LIBPATH ) ...LIB_PATH e.g. ${OPENCV_LIBS}
  TARGET_LINK_LIBRARIES(test_so3 ${Sophus_LIBRARIES})
  TARGET_LINK_LIBRARIES(test_se3 ${Sophus_LIBRARIES})
  TARGET_LINK_LIBRARIES(test_eigen ${Eigen3_LIBRARIES})
