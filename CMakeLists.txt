#cmake_minimum_required(VERSION 3.6)
cmake_minimum_required(VERSION 2.8)
project(faceDetectionNTracking)

set(CMAKE_CXX_STANDARD 14)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )
file(COPY haarcascade_frontalface_default.xml DESTINATION ${PROJECT_BINARY_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Curses REQUIRED) 
include_directories(${CURSES_INCLUDE_DIR})

set(SOURCE_FILES main.cpp VideoFaceDetector.cpp VideoFaceDetector.h)
add_executable(demo ${SOURCE_FILES})

target_link_libraries(demo ${OpenCV_LIBS} ${CURSES_LIBRARIES})
