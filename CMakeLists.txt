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

#SET(CUDA_INCLUDE_DIRS "/usr/local/cuda/include" ${CUDA_INCLUDE_DIRS})
#SET(CUDA_LIBRARIES ${CUDA_LIBRARIES} "/usr/local/cuda/lib64/libcudart.so" "/usr/lib64/libcuda.so" "/usr/local/cuda/lib64/libcublas.so")
set(CMAKE_CXX_FLAGS "-std=c++11")
cuda_add_executable (ESD GPU.cu ESD.cpp)
#set(CMAKE_CXX_FLAGS "-std=c++11")
#add_executable (ESD ESD.cpp)
#set(
#    CUDA_NVCC_FLAGS
#    ${CUDA_NVCC_FLAGS};
#    -O3 -gencode arch=compute_22,code=sm_22
#    )

#cuda_add_executable (ESD ESD.cpp)
#cuda_add_executable (ESD ESD.cu)
#FILE(GLOB SOURCES "*.cu")
#CUDA_ADD_EXECUTABLE(ESD ${SOURCES})

#LIST(APPEND CMAKE_CXX_FLAGS "-std=c++0x -O3 -ffast-math -Wall")

#LIST(APPEND CUDA_NVCC_FLAGS --compiler-options -fno-strict-aliasing -lineinfo -use_fast_math -Xptxas -dlcm=cg)
#LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20)
#LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_30,code=sm_30)
#LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_35,code=sm_35)
target_link_libraries (ESD ${PCL_LIBRARIES} /usr/local/cuda/lib64/libcudart.so)
#target_link_libraries (ESD${PCL_LIBRARIES}
#    -lfoobar
#    -ljoestuff)
#add_executable (extraPlanarRefinements extraPlanarRefinements.cpp)
#target_link_libraries (extraPlanarRefinements ${PCL_LIBRARIES})
#add_executable (viewerESD viewerESD.cpp)
#target_link_libraries (viewerESD ${PCL_LIBRARIES})
