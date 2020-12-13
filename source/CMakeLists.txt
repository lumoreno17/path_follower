#Cmake version
cmake_minimum_required(VERSION 3.5)

#Debug
set(CMAKE_BUILD_TYPE Debug)

#Project Name
project(path_follower)

#Include path
include_directories(../source)
include_directories(../include)

file(GLOB SOURCES
    ../include/*.h
    ../source/*.cpp
    ../source/*.c
)

file(GLOB MAIN1
    ../source/main.cpp
)
#Include OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_definitions(-DMAX_EXT_API_CONNECTIONS=255 -DNON_MATLAB_PARSING -DDO_NOT_USE_SHARED_MEMORY -DDEBUG)
add_definitions(-std=c++11)

add_executable(path_follower ${SOURCES})
target_link_libraries( path_follower ${OpenCV_LIBS} )
