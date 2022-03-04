cmake_minimum_required(VERSION 3.10)
project(depthai_tof_demo)

set(CMAKE_CXX_STANDARD 14)
find_package(depthai REQUIRED)

find_package( OpenCV REQUIRED COMPONENTS core highgui imgproc )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(depthai-tof-demo depthai-tof-demo.cc)
target_link_libraries(depthai-tof-demo depthai::opencv ${OpenCV_LIBS})
