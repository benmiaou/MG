#ifndef MESH_H
#define MESH_H

#include "Shape.h"

#include <surface_mesh/surface_mesh.h>

class Mesh : public Shape
{
public:
    Mesh() {}
    ~Mesh();
    void load(const std::string& filename);
    void init(Shader *shader);
    void draw(Shader *shader, bool drawEdges = false);

private:
    void find();
    GLuint mVao;
    GLuint mBufs[3];

    std::vector<Eigen::Vector3f> mPositions;
    std::vector<Eigen::Vector3f> mNormals;

    void specifyVertexData(Shader *shader);
    std::vector<Eigen::Vector3i> mIndices;
    std::vector<unsigned int> mValence;

    surface_mesh::Surface_mesh mHalfEdge;

    GLuint mIndicesBuffer;
};


#endif // MESH_H
