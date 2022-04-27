cmake_minimum_required(VERSION 3.14)

option(BUILD_DEPTHAI "Build all relevant dependencies" OFF)

include(FetchContent)
if(BUILD_DEPTHAI)
  # FetchContent_Declare(
  #       opencv
  #       GIT_REPOSITORY https://github.com/opencv/opencv.git
  #       GIT_TAG        4.4.0
  # 	)

  FetchContent_Declare(
    depthai-core
    GIT_REPOSITORY https://github.com/luxonis/depthai-core.git
    GIT_TAG tof_rgb_mono
    )

  #FetchContent_MakeAvailable(opencv)
  FetchContent_MakeAvailable(depthai-core)
else()
  find_package(depthai REQUIRED)

endif()
project(depthai_tof_demo)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(depthai-tof-demo depthai-tof-demo.cc)
target_link_libraries(depthai-tof-demo depthai::opencv ${OpenCV_LIBS})
