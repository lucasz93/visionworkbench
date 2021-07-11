if(APPLE)
    # Needed for conda build on OSX with conda
    cmake_policy(SET CMP0025 NEW) 
endif(APPLE)

cmake_minimum_required (VERSION 3.2)

# Make it possible to append to these from the command line
set(CMAKE_CXX_FLAGS "" CACHE STRING "")
set(CMAKE_C_FLAGS "" CACHE STRING "")

# Make sure we do a release type
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
endif()
message("Setting CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")

# Determine optimization level.
if(${CMAKE_BUILD_TYPE} STREQUAL "Release" OR ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo")
    set(OPTIMIZATION_LEVEL "-O3")
elseif(${CMAKE_BUILD_TYPE} STREQUAL "MinSizeRel")
    set(OPTIMIZATION_LEVEL "-Os")
else()
    set(OPTIMIZATION_LEVEL "-Og")
endif()

# cmake fails without the -L flag in CMAKE_CXX_FLAGS, oddly enough.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPTIMIZATION_LEVEL} -g -std=c++11 -L${BINARYBUILDER_INSTALL_DIR}/lib")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPTIMIZATION_LEVEL} -g")
set(CMAKE_EXE_LINKER_FLAGS "-lm -pthread -lpthread")

if(APPLE)
    # A workaround for the clang included with conda build
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mlinker-version=305")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mlinker-version=305")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mlinker-version=305")
else(APPLE)
    # On Linux need to link to rt
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")
endif(APPLE)

# Tell cmake to look in the /cmake folder.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

project(visionworkbench)

add_subdirectory(src)

