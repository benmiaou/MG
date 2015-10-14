#ifndef BPA_H
#define BPA_H
#include <vector>
#include "Pointcloud.h"
#include "Octree.h"
#include "Eigen/Geometry"
class BPA
{
private:
    struct Sphere{
        Sphere(double rad, Eigen::Vector3f c){
            radius = rad;
            center = c;
        }
        double radius;
        Eigen::Vector3f center;
    };

public:
    BPA(PointCloud model, Octree octree);
};

#endif // BPA_H
