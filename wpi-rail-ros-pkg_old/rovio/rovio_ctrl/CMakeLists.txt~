cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(rovio_move src/rovio_move.cpp)
rosbuild_add_executable(rovio_head src/rovio_head.cpp)
rosbuild_add_executable(rovio_teleop src/rovio_teleop.cpp)

rosbuild_gensrv()
