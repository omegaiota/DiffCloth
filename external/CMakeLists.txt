# CMake entry point.
cmake_minimum_required(VERSION 3.1.0)

# Do not want to generate zeor_check because they cause messy folders in
# visual studio 2017.
set(CMAKE_SUPPRESS_REGENERATION true)

set(CMAKE_CXX_STANDARD 11)

set(EXTERNAL_HEADER
        "${CMAKE_CURRENT_LIST_DIR}/eigen/"
        )

include_directories(${EXTERNAL_HEADER})

