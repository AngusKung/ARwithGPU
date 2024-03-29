#include <stdint.h>
#pragma once

class supervoxel {
public:
    supervoxel( uint32_t num = 0, int i = 0, bool u = 0) :
        cluster_int(i), cluster_num(num), used(u) {
        plane_id = -1;
    }


    void setNeighbors(std::vector<supervoxel*> *sv_ptr){
        n_size = sv_ptr->size();
        neighbors = new supervoxel*[n_size];
        std::vector<supervoxel*>::iterator it = sv_ptr->begin();
        for(int i = 0; i != n_size; ++i) {
            neighbors[i] = *it;
            ++it;
        }
    }

    int      plane_id;
    uint32_t cluster_num;
    int      cluster_int;
    bool     used;
    int* neighbors;
    int          n_size;
};
