#pragma once

class planeVoxel {
public:
    planeVoxel(const uint32_t nu = 0, const int i = 0) : num(nu), id(i) {
        next = NULL;
    }

    planeVoxel* next;
    uint32_t    num;
    int         id;
};
