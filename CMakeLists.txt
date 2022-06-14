cmake_minimum_required(VERSION 3.14)

set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO "RelWithDebInfo;Release;")

option(BUILD_DEPTHAI "Build all relevant dependencies" OFF)

include(FetchContent)
if(BUILD_DEPTHAI)
  FetchContent_Declare(
    depthai-core
    GIT_REPOSITORY https://github.com/constructiverealities/depthai.git
    GIT_TAG 2583d4d9e05ca355d13a8cbc7c3055a81e41f62b
    )

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

if(WIN32)
  install(IMPORTED_RUNTIME_ARTIFACTS ${OpenCV_LIBS})  
  install(FILES depthai-tof-demo-5fps.bat depthai-tof-demo-30fps.bat DESTINATION bin)
  install(FILES $<TARGET_PDB_FILE:depthai-tof-demo> DESTINATION bin OPTIONAL)
endif()
