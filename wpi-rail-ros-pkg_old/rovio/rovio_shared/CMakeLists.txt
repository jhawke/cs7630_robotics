cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#generate messages
rosbuild_genmsg()
#generate services
rosbuild_gensrv()

#find the CURL libraries
include(FindCURL)

rosbuild_add_library(${PROJECT_NAME} src/rovio_http.cpp)
target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES})
