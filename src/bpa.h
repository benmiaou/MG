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
    struct BPATriangle{
      BPATriangle(Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3){
    vertex1 = v1;
    vertex2 = v2;
    vertex3 = v3;
      }
      Eigen::Vector3f vertex1;
      Eigen::Vector3f vertex2;
      Eigen::Vector3f vertex3;
    };
    BPATriangle *actualTriangle;
    std::vector<Eigen::Vector3f> edges;

    BPASphere *actualSphere;
    int findNext(Eigen::Vector3f p1, Eigen::Vector3f p2, BPASphere *actualSphere, Octree *octree);
    void moveSphere(double theta, Eigen::Vector3f axis);
    void findNewTriangle(double theta, Eigen::Vector3f axis);
    Eigen::Vector3f findPoint(std::vector<Eigen::Vector3f> tab);
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
