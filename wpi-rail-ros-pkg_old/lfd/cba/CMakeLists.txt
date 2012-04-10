cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(cba src/cba.cpp)
rosbuild_add_executable(mock_state_publisher src/test/mock_state_publisher.cpp)
rosbuild_add_executable(mock_classify src/test/mock_classify.cpp)
rosbuild_add_executable(mock_agent src/test/mock_agent.cpp)
