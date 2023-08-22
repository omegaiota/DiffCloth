cmake_minimum_required(VERSION 3.15)
project(DiffCloth)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "set env var")

# Properly setting environment variables for Apple Clang
if (APPLE)
    include_directories("/usr/local/include" "/usr/local/opt/llvm/include" "/opt/homebrew/opt/libomp/include")
    link_directories("/usr/local/lib" "/usr/local/opt/llvm/lib" "/opt/homebrew/opt/libomp/lib")

    set(ENV{CC} "/usr/local/opt/llvm/bin/clang")
    set(ENV{CXX} "/usr/local/opt/llvm/bin/clang-omp++")
    set(ENV{LDFLAGS} "-L/usr/local/opt/libomp/lib")
    set(ENV{CPPFLAGS} "-I/usr/local/opt/libomp/include")

    # Set OpenMP flags and libraries before find_package
    set(OpenMP_C "${CMAKE_C_COMPILER}")
    set(OpenMP_C_FLAGS "-Xclang -fopenmp -I/usr/local/Cellar/libomp/16.0.6/include")
    set(OpenMP_C_LIB_NAMES "libomp")
    set(OpenMP_libomp_LIBRARY "omp")

    set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
    set(OpenMP_CXX_FLAGS "-Xclang -fopenmp -I/usr/local/Cellar/libomp/16.0.6/include")
    set(OpenMP_CXX_LIB_NAMES "libomp")
    set(OpenMP_libomp_LIBRARY "omp")
    set(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib")

    link_directories("/usr/local/Cellar/libomp/16.0.6/lib/")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_NAME DiffCloth)
set(EXTERNAL_ROOT ${PROJECT_SOURCE_DIR}/external)
set(LIB dependencies/lib)

add_definitions(-DSOURCE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")

add_definitions(-DCMAKE_TOOLCHAIN_FILE=${PROJECT_SOURCE_DIR}/external/vcpkg/scripts/buildsystems/vcpkg.cmake)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(OpenMP_C "${CMAKE_C_COMPILER}")
 	set(OpenMP_C_FLAGS "-Xclang -fopenmp -I/usr/local/Cellar/libomp/16.0.6/include")
 	set(OpenMP_C_LIB_NAMES "libomp")
 	set(OpenMP_libomp_LIBRARY "omp")

 	set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
 	set(OpenMP_CXX_FLAGS "-Xclang -fopenmp -I/usr/local/Cellar/libomp/16.0.6/include")
 	set(OpenMP_CXX_LIB_NAMES "libomp")
 	set(OpenMP_libomp_LIBRARY "omp")
    set(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib")

 	link_directories("/usr/local/Cellar/libomp/16.0.6/lib/")
endif()

#if (APPLE)
#    add_compile_options(-DUSING_MACBOOK)
#endif ()

if (USE_DEBUG)
    add_compile_options(-DUSE_DEBUG)
endif ()


message("cxx Flags: " ${CMAKE_CXX_FLAGS})

source_group("Shaders" FILES src/gpu/shaders)
source_group("Assets" FILES src/assets)

add_subdirectory(${EXTERNAL_ROOT})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)








###############
get_directory_property(EXTERNAL_HEADER
        DIRECTORY ${EXTERNAL_ROOT}
        DEFINITION EXTERNAL_HEADER)

include_directories(dependencies/include)
include_directories(src/assets)
include_directories(src/gpu)
include_directories(external/LBFGSpp/include)

include_directories(dependencies/lib)


link_directories(${LIB})

# OmegaEngine cpp
file(GLOB HEADERS "src/code/*.h" "src/code/engine/*.h" "src/code/simulation/*.h" "src/code/supports/*.h" "src/code/optimization/*.h")
file(GLOB SRCFILES "src/code/main.cpp" "src/code/*.h" "src/code/*.c" "src/code/engine/*.cpp" "src/code/engine/*.h" "src/code/simulation/*.cpp" "src/code/simulation/*.h" "src/code/supports/*.cpp" "src/code/supports/*.h" "src/code/optimization/*.cpp" "src/code/optimization/*.h")
source_group("Sources" FILES ${SRCFILES})
add_executable(${EXECUTABLE_NAME} ${SRCFILES}  )

target_link_directories(${PROJECT_NAME} PRIVATE "/usr/local/Cellar/libomp/16.0.6/lib/")
target_link_libraries(${EXECUTABLE_NAME} OpenMP::OpenMP_CXX)
target_include_directories(${EXECUTABLE_NAME} PRIVATE ${EXTERNAL_HEADER})





