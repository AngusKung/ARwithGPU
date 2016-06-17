#include <vector>
//#include <map>
#include "planeObject.h"
#include "supervoxel.h"
#include "planeVoxel.h"
#include "planesVector.h"
#include "SyncedMemory.h"
//cuda
#include <thrust/device_vector.h>
//#include <thrust/copy.h>
#define EXE_PER_THREAD 100
#define parrallel_threshold 0.8
#define mu 0.2
#define parrallel_filter 0.8
#define distance_to_plane 0.005

__device__ void push_back_voxel(planeObject& o, planeVoxel& v) {
    planeVoxel* ptr = o.begin;
    if(ptr == NULL) {
        ptr = &v;
        o.size++;
        return;
    }
    while(ptr->next != NULL) {
        ptr = ptr->next;
    }
    ptr->next = &v;
    o.size++;
    return;
}

__device__ void push_back_plane(planesVector& v, planeObject& o) {
    planeObject* ptr = v.begin;
    if(ptr == NULL) {
        ptr = &o;
        v.size++;
        return;
    }
    while(ptr->next != NULL) {
        ptr = ptr->next;
    }
    ptr->next = &o;
    v.size++;
    return;
}

__device__ void append(planeObject& be_appended, planeObject& to_append) {
    planeVoxel* end = be_appended.begin;
    while(end->next != NULL) end = end->next;
    end->next = to_append.begin;
    be_appended.size += to_append.size;
}

__device__ void
findNeighbor(planeObject& plane, supervoxel* v_ptr,
             double& the_normal_x, double& the_normal_y, double& the_normal_z,
             double* normal_vector_x, double* normal_vector_y, double* normal_vector_z)
{
  planeVoxel temp(v_ptr->cluster_num, v_ptr->cluster_int);
  push_back_voxel(plane, temp );

//==========ORIGINAL CODE===============
  for(int i = 0; i != v_ptr->n_size; ++i) {
    supervoxel* neighbor = v_ptr->neighbors[i];
    int neighbor_cluster_int = neighbor->cluster_int;
    // Check whether the neighbor has normals like a plane
    // Supervoxel has normal of 1
    if(the_normal_x * normal_vector_x[neighbor_cluster_int] + the_normal_y * normal_vector_y[neighbor_cluster_int] +
        the_normal_z * normal_vector_z[neighbor_cluster_int] > parrallel_threshold && neighbor->used == false){
      neighbor->used = true;
      the_normal_x = (1-mu)*the_normal_x+mu*normal_vector_x[neighbor_cluster_int];
      the_normal_y = (1-mu)*the_normal_y+mu*normal_vector_y[neighbor_cluster_int];
      the_normal_z = (1-mu)*the_normal_z+mu*normal_vector_z[neighbor_cluster_int];
      findNeighbor(plane, neighbor,the_normal_x,the_normal_y,the_normal_z,
                   normal_vector_x, normal_vector_y, normal_vector_z);
    }
  }
  return;
//====================================

/*==============fake code=============
  for(int i = 0; i != 7; ++i) {
    uint32_t neighbor_cluster = i;
    int neighbor_cluster_int = i;
    // Check whether the neighbor has normals like a plane
    // Supervoxel has normal of 1
    //double adj_parrallel_threshold = parrallel_threshold * (1+(pos_z[neighbor_cluster_int] - min_z));
    if(the_normal_x * normal_vector_x[neighbor_cluster_int] + the_normal_y * normal_vector_y[neighbor_cluster_int] +
        the_normal_z * normal_vector_z[neighbor_cluster_int] > parrallel_threshold ){
      the_normal_x = (1-mu)*the_normal_x+mu*normal_vector_x[neighbor_cluster_int];
      the_normal_y = (1-mu)*the_normal_y+mu*normal_vector_y[neighbor_cluster_int];
      the_normal_z = (1-mu)*the_normal_z+mu*normal_vector_z[neighbor_cluster_int];
      //findNeighbor(plane, ,the_normal_x,the_normal_y,the_normal_z,
        //           normal_vector_x, normal_vector_y, normal_vector_z);
                   //supervoxel_adjacency, clusters_int, clusters_used);
    }
  }
======================================*/
}

__global__ void labelWithGPU(supervoxel* voxels, int v_size,
                             double* normal_vector_x,
                             double* normal_vector_y,
                             double* normal_vector_z,
                             double* pos_x,
                             double* pos_y,
                             double* pos_z,
                             planesVector* p_v) {
  int base = threadIdx.x*EXE_PER_THREAD;
  int the_cluster_int;
  int size_temp;
  double the_normal_x, the_normal_y, the_normal_z, avn_x, avn_y, avn_z, avp_x, avp_y, avp_z; 
  planesVector planesVectors = p_v[threadIdx.x];
  for(int i = base; i != EXE_PER_THREAD+base; ++i)
  {
    if( i >= v_size ) break;
    if( voxels[i].used==true ) continue;
    voxels[i].used = true;
    the_cluster_int = voxels[i].cluster_int;
    the_normal_x = normal_vector_x[the_cluster_int];
    the_normal_y = normal_vector_y[the_cluster_int];
    the_normal_z = normal_vector_z[the_cluster_int];

    planeObject plane;
    findNeighbor(plane, &voxels[i], the_normal_x,the_normal_y,the_normal_z,
                 normal_vector_x, normal_vector_y, normal_vector_z);
    size_temp = plane.size;//planeObject pointer's
    if(size_temp <= 1)
      continue;
    avp_x = 0; avp_y = 0; avp_z = 0; avn_x = 0; avn_y = 0; avn_z = 0;
    planeVoxel* pv_it = plane.begin;
    while(pv_it != NULL) {
      the_cluster_int = pv_it->id;
      avn_x += normal_vector_x[the_cluster_int];
      avn_y += normal_vector_y[the_cluster_int];
      avn_z += normal_vector_z[the_cluster_int];
      avp_x += pos_x[the_cluster_int];
      avp_y += pos_y[the_cluster_int];
      avp_z += pos_z[the_cluster_int];
      pv_it = pv_it->next;
    }
    avn_x /= double(size_temp);
    avn_y /= double(size_temp);
    avn_z /= double(size_temp);
    avp_x /= double(size_temp);
    avp_y /= double(size_temp);
    avp_z /= double(size_temp);

        bool new_plane = true;
        //Planar Refinements
    for(int j = 0; j != blockDim.x; ++j) {
        planeObject* po_ptr = p_v[j].begin;
        while(po_ptr != NULL) {
            const double on_x = po_ptr->aver_nor_x;
            const double on_y = po_ptr->aver_nor_y;
            const double on_z = po_ptr->aver_nor_z;
            const double op_x = po_ptr->aver_pos_x;
            const double op_y = po_ptr->aver_pos_y;
            const double op_z = po_ptr->aver_pos_z;
            if(std::abs(avn_x*on_x +avn_y*on_y +avn_z*on_z) > parrallel_filter && 
               std::abs((avn_x*avp_x + avn_y*avp_y + avn_z*avp_z) - (on_x*op_x + on_y*op_y + on_z*op_z)) < distance_to_plane ) {
                new_plane = false;
                double weight = size_temp / double(size_temp + po_ptr->size);
                po_ptr->aver_nor_x = (1-weight)*on_x + weight*avn_x ;
                po_ptr->aver_nor_y = (1-weight)*on_y + weight*avn_y ;
                po_ptr->aver_nor_z = (1-weight)*on_z + weight*avn_z ;
                po_ptr->aver_pos_x = (1-weight)*op_x + weight*avp_x ;
                po_ptr->aver_pos_y = (1-weight)*op_y + weight*avp_y ;
                po_ptr->aver_pos_z = (1-weight)*op_z + weight*avp_z ;
                append(*po_ptr, plane);
                break;
            }
        po_ptr = po_ptr->next;
        }
        if(new_plane == false) break;
    }
        if(new_plane == true){
        plane.aver_nor_x = avn_x;
        plane.aver_nor_y = avn_y;
        plane.aver_nor_z = avn_z;
        plane.aver_pos_x = avp_x;
        plane.aver_pos_y = avp_y;
        plane.aver_pos_z = avp_z;
        
        push_back_plane( planesVectors, plane );
        }
  }
}

void copyPlaneToHost(planesVector* planesVectors_gpu, int numOfThreads, std::vector<planeObject>& planesVectors) {
    for(int i = 0; i != numOfThreads; ++i) {
        planeObject* ptr = planesVectors_gpu[i].begin;
        while(ptr->next != NULL) {
            planesVectors.push_back(*ptr);
            ptr = ptr->next;
        }
    }
}

__global__ void test_t(t) {
    int hi = 5;
    for(test* i = t->)
}

void gpu(const std::vector<supervoxel>& voxels,
         const std::vector<double>& normal_vector_x,
         const std::vector<double>& normal_vector_y,
         const std::vector<double>& normal_vector_z,
         const std::vector<double>& pos_x,
         const std::vector<double>& pos_y,
         const std::vector<double>& pos_z,
         std::vector<planeObject>& planesVectors
) {
/*//test code======
  MemoryBuffer<test> t(5);
  auto t_sync = t.CreateSync(5);
  test* t_gpu = t_sync.get_gpu_rw();
  test_t<<<1,1>>>(t_gpu);


//================
*/

  int v_size;//nvx_size, nvy_size, nvz_size, px_size, py_size, pz_size, pv_size;
  //read only
  thrust::device_vector<supervoxel> voxels_gpu_v(voxels.begin(), voxels.end());
  supervoxel* voxels_gpu = thrust::raw_pointer_cast(&voxels_gpu_v[0]);
  v_size = voxels_gpu_v.size();
/*
  size_t s_a_size = sizeOfMultiMap(supervoxel_adjacency);
  std::multimap<uint32_t, uint32_t>* supervoxel_adjacency_cpu = &supervoxel_adjacency;
  std::multimap<uint32_t, uint32_t>* supervoxel_adjacency_gpu;
  cudaMalloc((void**) &supervoxel_adjacency_gpu, s_a_size);
  cudaMemcpy(supervoxel_adjacency_gpu, &supervoxel_adjacency, s_a_size, cudaMemcpyHostToDevice);
*/
  thrust::device_vector<double> normal_vector_x_gpu_v(normal_vector_x.begin(), normal_vector_x.end());
  double* normal_vector_x_gpu = thrust::raw_pointer_cast(&normal_vector_x_gpu_v[0]);
  thrust::device_vector<double> normal_vector_y_gpu_v(normal_vector_y.begin(), normal_vector_y.end());
  double* normal_vector_y_gpu = thrust::raw_pointer_cast(&normal_vector_y_gpu_v[0]);
  thrust::device_vector<double> normal_vector_z_gpu_v(normal_vector_z.begin(), normal_vector_z.end());
  double* normal_vector_z_gpu = thrust::raw_pointer_cast(&normal_vector_z_gpu_v[0]);

  thrust::device_vector<double> pos_x_gpu_v(pos_x.begin(), pos_x.end());
  double* pos_x_gpu = thrust::raw_pointer_cast(&pos_x_gpu_v[0]);
  thrust::device_vector<double> pos_y_gpu_v(pos_y.begin(), pos_y.end());
  double* pos_y_gpu = thrust::raw_pointer_cast(&pos_y_gpu_v[0]);
  thrust::device_vector<double> pos_z_gpu_v(pos_z.begin(), pos_z.end());
  double* pos_z_gpu = thrust::raw_pointer_cast(&pos_z_gpu_v[0]);
  //read and write
  //thrust::device_vector<planeObject> planesVectors_gpu_v;
  //planeObject* planesVectors_gpu = thrust::raw_pointer_cast(&planesVectors_gpu_v[0]);
//  pv_size = planesVectors_gpu_v.size();
  //start labeling
  int numOfThreads = ( v_size % EXE_PER_THREAD)? v_size/EXE_PER_THREAD+1: v_size/EXE_PER_THREAD;

  MemoryBuffer<planesVector> pv(numOfThreads);
  auto pv_sync = pv.CreateSync(numOfThreads);
  planesVector* planesVectors_cpu = pv_sync.get_cpu_rw();
  planesVector* planesVectors_gpu = pv_sync.get_gpu_rw();
 // cudaMalloc((void**) &planesVectors_gpu, numOfThreads);
 // cudaMemcpy(planesVectors_gpu, planesVectors_cpu, numOfThreads, cudaMemcpyHostToDevice);
  labelWithGPU<<<1, numOfThreads>>>(voxels_gpu, v_size,
                                    normal_vector_x_gpu,  normal_vector_y_gpu, normal_vector_z_gpu,
                                    pos_x_gpu, pos_y_gpu, pos_z_gpu, planesVectors_gpu);
 // cudaMemcpy(planesVectors_cpu, planesVectors_gpu, numOfThreads, cudaMemcpyDeviceToHost);
  copyPlaneToHost(planesVectors_cpu, numOfThreads, planesVectors);
  //thrust::copy(planesVectors_gpu_v.begin(), planesVectors_gpu_v.end(), planesVectors.begin());
}
