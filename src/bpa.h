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
    struct Edge{
      Edge(int idp1,int idp2, Eigen::Vector3f center){
          id1 = idp1;
          id2 = idp2;
          sphereCenter = center;
      }
      int id1;
      int id2;
      Eigen::Vector3f sphereCenter;
    };
    std::vector<Edge> edges;

    BPASphere *actualSphere;
    int findNext(Eigen::Vector3f p1, Eigen::Vector3f p2, BPASphere *actualSphere, Octree *octree);
    void moveSphere(double theta, Eigen::Vector3f axis);
    Eigen::Vector3f findPoint(std::vector<Eigen::Vector3f> tab);
    int getSeed(Octree *octree, std::vector<bool> &isVisited,  std::vector<Eigen::Vector3f> &positions, std::vector<int> &ids, double &r);


public:
    void init(Shader *shader);
    void draw(Shader *shader, bool drawEdges = false);

    BPA(PointCloud *model, Octree *octree);
    double getRadius();
    Eigen::Vector3f getCenter();


    void specifyVertexData(Shader *shader);
    GLuint mVao;
    GLuint mBufs[3];

    std::vector<Eigen::Vector3i> mIndices;
    std::vector<Eigen::Vector3f>	mVertices;
    std::vector<Eigen::Vector3f> mNormals;
    GLuint mIndicesBuffer;

};

#endif // BPA_H
