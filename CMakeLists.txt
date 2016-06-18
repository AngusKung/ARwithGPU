cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ESD)

find_package(PCL 1.7 REQUIRED)
#find_package(CUDA QUIET REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#copied from the internet by cosine
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()



#include_directories(${CUDA_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CUDA_NVCC_FLAGS "-std=c++11 -g -G")
cuda_add_executable (ESD GPU.cu ESD.cpp)
target_link_libraries (ESD ${PCL_LIBRARIES} /usr/local/cuda/lib64/libcudart.so)

