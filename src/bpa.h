#ifndef BPA_H
#define BPA_H
#include <vector>
#include "Pointcloud.h"
#include "Octree.h"
#include "Eigen/Geometry"

class BPA : public Shape
{
private:
    struct BPASphere {
        BPASphere(double rad, Eigen::Vector3f c){
            radius = rad;
            center = c;
        }
        double radius;
        Eigen::Vector3f center;
    };
    BPASphere *actualSphere;
public:
    void init(Shader *shader);
    void draw(Shader *shader, bool drawEdges = false);

    BPA(PointCloud *model, Octree *octree);
    double getRadius();
    Eigen::Vector3f getCenter();

    void specifyVertexData(Shader *shader);
    GLuint mVao;
    GLuint mBufs[2];

    std::vector<int>        mIndices;   /** vertex indices */
    std::vector<Eigen::Vector3f>	mVertices;  /** 3D positions */
};

#endif // BPA_H
