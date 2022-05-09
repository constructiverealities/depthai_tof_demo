cmake_minimum_required(VERSION 3.14)

option(BUILD_DEPTHAI "Build all relevant dependencies" OFF)

include(FetchContent)
if(BUILD_DEPTHAI)
  FetchContent_Declare(
    depthai-core
    GIT_REPOSITORY https://github.com/luxonis/depthai-core.git
    GIT_TAG tof_rgb_mono
    )

  #FetchContent_MakeAvailable(opencv)
  FetchContent_MakeAvailable(depthai-core)
  project(depthai_tof_demo)
else()
  project(depthai_tof_demo)
  find_package(depthai REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(depthai-tof-demo depthai-tof-demo.cc)
target_link_libraries(depthai-tof-demo depthai::opencv ${OpenCV_LIBS})
install(TARGETS depthai-tof-demo DESTINATION bin)
install(IMPORTED_RUNTIME_ARTIFACTS ${OpenCV_LIBS})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/mvcmds DESTINATION bin)

if(WIN32)
  install(FILES depthai-tof-demo-5fps.bat depthai-tof-demo-30fps.bat DESTINATION bin)
endif()