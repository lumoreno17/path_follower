#Cmake version
cmake_minimum_required(VERSION 3.8)

#Debug
set(CMAKE_BUILD_TYPE Debug)

#Project Name
project(robitca_movel)

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

add_executable(robitca_movel ${SOURCES})
target_link_libraries( robitca_movel ${OpenCV_LIBS} )