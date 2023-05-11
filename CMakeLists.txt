cmake_minimum_required(VERSION 3.14)

set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO "RelWithDebInfo;Release;")

option(BUILD_DEPTHAI "Build all relevant dependencies" OFF)

include(FetchContent)
if(BUILD_DEPTHAI)
  FetchContent_Declare(
    depthai-core
    GIT_REPOSITORY https://github.com/constructiverealities/depthai.git
    GIT_TAG e48fd69b5ff5e9923de4330cd9c6b42d85fd8374
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
install(TARGETS depthai-tof-demo DESTINATION bin RUNTIME_DEPENDENCY_SET dependencies)

if(WIN32)
  install(IMPORTED_RUNTIME_ARTIFACTS ${OpenCV_LIBS})  
  install(FILES depthai-tof-demo-5fps.bat depthai-tof-demo-30fps.bat DESTINATION bin RUNTIME_DEPENDENCY_SET dependencies)
  install(FILES $<TARGET_PDB_FILE:depthai-tof-demo> DESTINATION bin OPTIONAL RUNTIME_DEPENDENCY_SET dependencies)
endif()

install(RUNTIME_DEPENDENCY_SET dependencies)

find_package(libobs)
if (EXISTS ${LIBOBS_INCLUDE_DIR}/../cmake/external/ObsPluginHelpers.cmake)
  include(${LIBOBS_INCLUDE_DIR}/../cmake/external/ObsPluginHelpers.cmake)
endif()

if(libobs_FOUND)
message("Have OBS")
endif()

INSTALL(CODE
  "include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/appimage.cmake)
  make_appimage(
  EXE \"${CMAKE_INSTALL_PREFIX}/bin/depthai-tof-demo\"
  NAME \"depthai-tof-demo\"
  ICON \"${CMAKE_CURRENT_SOURCE_DIR}/icons/cr-logo-only-blk.png\"
  DIR_ICON \"${CMAKE_CURRENT_SOURCE_DIR}/icons/cr-logo-only-blk.png\"
  OUTPUT_NAME \"${CMAKE_CURRENT_BINARY_DIR}/depthai-tof-demo.AppImage\"
  )
  "
COMPONENT Runtime
)
