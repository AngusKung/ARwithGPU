//#include <map>
#include <vector>
/*
__device__ void findNeighbor(supervoxel* v_ptr,
             double& the_normal_x, double& the_normal_y, double& the_normal_z,
             double* normal_vector_x, double* normal_vector_y, double* normal_vector_z){};

__global__ void labelWithGPU(supervoxel* voxels, int v_size,
                             //std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                             //std::map<uint32_t, int>& clusters_int,
                             //std::map<uint32_t, bool>& clusters_used,
                             double* normal_vector_x,
                             double* normal_vector_y,
                             double* normal_vector_z,
                             double* pos_x,
                             double* pos_y,
                             double* pos_z,
                             planeObject* planesVectors){};

void copyPlaneToHost(thrust::device_vector<planeObject>& planesVectors){};
*/
void gpu(const std::vector<supervoxel>& voxels,
         const std::vector<double>& normal_vector_x,
         const std::vector<double>& normal_vector_y,
         const std::vector<double>& normal_vector_z,
         const std::vector<double>& pos_x,
         const std::vector<double>& pos_y,
         const std::vector<double>& pos_z,
         std::vector<planeObject>& planesVectors);
