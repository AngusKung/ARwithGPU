#include "planeVoxel.h"
#pragma once

class planeObject {
public:

    planeObject(planeVoxel* b = NULL, const int& s = 0, const double& nx = 0, const double& ny = 0, const double& nz = 0,
                const double& px = 0, const double& py = 0, const double& pz = 0) :
                begin(b), size(s), aver_nor_x(nx), aver_nor_y(ny), aver_nor_z(nz),
                aver_pos_x(px), aver_pos_y(py), aver_pos_z(pz) {
        next = NULL;
    }
/*
    double get_aver_nor_x() { return aver_nor_x; }
    double get_aver_nor_y() { return aver_nor_y; }
    double get_aver_nor_z() { return aver_nor_z; }
    double get_aver_pos_x() { return aver_pos_x; }
    double get_aver_pos_y() { return aver_pos_y; }
    double get_aver_pos_z() { return aver_pos_z; }

    void set_aver_nor_x(const double& x) { aver_nor_x = x; }
    void set_aver_nor_y(const double& y) { aver_nor_y = y; }
    void set_aver_nor_z(const double& z) { aver_nor_z = z; }
    void set_aver_pos_x(const double& x) { aver_pos_x = x; }
    void set_aver_pos_y(const double& y) { aver_pos_y = y; }
    void set_aver_pos_z(const double& z) { aver_pos_z = z; }
*/
    double aver_nor_x, aver_nor_y, aver_nor_z, aver_pos_x, aver_pos_y, aver_pos_z;
    planeObject* next;

    planeVoxel* begin;
   // planeVoxel* end;
    int         size;
};
