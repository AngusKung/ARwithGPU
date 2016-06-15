cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ESD)

find_package(PCL 1.7 REQUIRED)
#find_package(CUDA QUIET REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories(${CUDA_INCLUDE_DIRS})

#SET(CUDA_INCLUDE_DIRS "/usr/local/cuda/include" ${CUDA_INCLUDE_DIRS})
#SET(CUDA_LIBRARIES ${CUDA_LIBRARIES} "/usr/local/cuda/lib64/libcudart.so" "/usr/lib64/libcuda.so" "/usr/local/cuda/lib64/libcublas.so")
set(CMAKE_CXX_FLAGS "-std=c++0x")
#set(
#    CUDA_NVCC_FLAGS
#    ${CUDA_NVCC_FLAGS};
#    -O3 -gencode arch=compute_22,code=sm_22
#    )

cuda_add_executable (ESD ESD.cu)
target_link_libraries (ESD ${PCL_LIBRARIES} /usr/local/cuda/lib64/libcudart.so)
#target_link_libraries (ESD${PCL_LIBRARIES}
#    -lfoobar
#    -ljoestuff)
#add_executable (extraPlanarRefinements extraPlanarRefinements.cpp)
#target_link_libraries (extraPlanarRefinements ${PCL_LIBRARIES})
#add_executable (viewerESD viewerESD.cpp)
#target_link_libraries (viewerESD ${PCL_LIBRARIES})
