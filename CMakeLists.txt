cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

add_definitions(-Wall)

#...: dynamic reconfigure :.....................................................

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#...: PMD SDK :.................................................................

if((CMAKE_SIZEOF_VOID_P MATCHES 4) OR (CMAKE_CL_64 MATCHES 0))
  set(PLUGIN_NAMES camboardnano.L32.pap camboardnanoproc.L32.ppp)
elseif((CMAKE_SIZEOF_VOID_P MATCHES 8) OR (CMAKE_CL_64 MATCHES 1))
  set(PLUGIN_NAMES camboardnano.L64.pap camboardnanoproc.L64.ppp)
else()
  message(SEND_ERROR "Unable to determine whether the OS is 32 or 64 bit.")
endif()

find_path(PMD_INCLUDE_DIR pmdsdk2.h
  $ENV{PMDDIR}/include
  /usr/local/pmd/include
)

find_library(PMD_LIBRARY pmdaccess2
  $ENV{PMDDIR}/bin
  /usr/local/pmd/bin
  $ENV{PMDDIR}/lib
  /usr/local/pmd/lib
)

find_path(PMD_PLUGIN_DIR ${PLUGIN_NAMES}
  $ENV{PMDDIR}/plugins
  /usr/local/pmd/plugins
)

if(NOT PMD_INCLUDE_DIR OR NOT PMD_LIBRARY OR NOT PMD_PLUGIN_DIR)
  message(SEND_ERROR "Could not find PMD SDK!")
endif(NOT PMD_INCLUDE_DIR OR NOT PMD_LIBRARY OR NOT PMD_PLUGIN_DIR)

include_directories(${PMD_INCLUDE_DIR})

# We need to pass the found directory of PMD plugins to the C++ compiler
# through a macro definition. Cmake changed the way quotes are treated in
# `add_definitions` somewhere in between 2.8.7 and 2.8.9, so we take different
# branches based on the version.
# (See https://github.com/taketwo/pmd_camboard_nano/issues/5)
if(CMAKE_MAJOR_VERSION GREATER 1 AND
   CMAKE_MINOR_VERSION GREATER 7 AND
   CMAKE_PATCH_VERSION GREATER 6 )
  add_definitions(-DPMD_PLUGIN_DIR="${PMD_PLUGIN_DIR}/")
else()
  add_definitions(-DPMD_PLUGIN_DIR=\\"${PMD_PLUGIN_DIR}/\\")
endif()

#...: default paths  :..........................................................

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

#...: target libraries :........................................................

#...: pmd_camboard_nano_nodelet
rosbuild_add_library(pmd_camboard_nano_nodelet
  src/driver_nodelet.cpp
  src/pmd_camboard_nano.cpp
)
target_link_libraries(pmd_camboard_nano_nodelet
  ${PMD_LIBRARY}
)

#...: target executables :......................................................

#...: driver_node
rosbuild_add_executable(pmd_camboard_nano_node
  src/driver_node.cpp
)
