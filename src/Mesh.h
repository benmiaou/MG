#ifndef MESH_H
#define MESH_H

#include "Shape.h"
#include "Pointcloud.h"
#include <surface_mesh/surface_mesh.h>
using namespace surface_mesh;

class Mesh : public PointCloud //modif Shape
{
public:
    Mesh() {}
    ~Mesh();
    void load(const std::string& filename);
    void init(Shader *shader);
    void draw(Shader *shader, bool drawEdges = false);

private:
    struct Hole{
      Hole(std::vector<Surface_mesh::Edge> edges){
          //bonding box, nb vertex, convex.

      }
    std::vector<Surface_mesh::Edge> edges;
    };

    void find();
    GLuint mVao;
    //GLuint mBufs[4];
    //GLuint mBufs[3];

    //std::vector<Eigen::Vector3f> mPositions;
    //std::vector<Eigen::Vector3f> mNormals;
    //std::vector<Eigen::Vector3f> mColors;

    //void specifyVertexData(Shader *shader);
    std::vector<Eigen::Vector3i> mIndices;
    std::vector<unsigned int> mValence;

    surface_mesh::Surface_mesh mHalfEdge;

    GLuint mIndicesBuffer;
};


#endif // MESH_H
