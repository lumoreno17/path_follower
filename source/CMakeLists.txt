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
)
#Include OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(path_follower ${SOURCES})
target_link_libraries( path_follower ${OpenCV_LIBS} )