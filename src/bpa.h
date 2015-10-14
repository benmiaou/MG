#ifndef BPA_H
#define BPA_H
#include <vector>
#include "Pointcloud.h"
#include "Octree.h"
#include "Eigen/Geometry"

class BPA
{
private:
    struct BPASphere{
        BPASphere(double rad, Eigen::Vector3f c){
            radius = rad;
            center = c;
        }
        double radius;
        Eigen::Vector3f center;
    };
    BPASphere *actualSphere;
public:

    BPA(PointCloud *model, Octree *octree);
    double getRadius();
    Eigen::Vector3f getCenter();
};

#endif // BPA_H
