#ifndef MESH_H
#define MESH_H

#include "Shape.h"
#include "Pointcloud.h"
#include <surface_mesh/surface_mesh.h>
#include <Eigen/Geometry>

using namespace surface_mesh;

class Mesh : public PointCloud //modif Shape
{
public:
    Mesh() {}
    ~Mesh();
    void load(const std::string& filename);
    void init(Shader *shader);
    void draw(Shader *shader, bool drawEdges = false);

    std::vector<Eigen::Vector3f> mColorFaces;
        std::vector<Eigen::Vector3f> mColorVal;
        std::vector<Eigen::Vector3f> mColorHoles;

    void fillAllHoles();
     void find();
    std::vector<Eigen::AlignedBox3f> getAABBs();
    float EPSI = 0.09;
private:

    struct Hole{
      Hole(){
      }
    std::vector<Surface_mesh::Halfedge> edges;
    Eigen::Vector3f max;
    Eigen::Vector3f min;
    Eigen::Vector3f geocenter;
    bool convex;
    };


    GLuint mVao;
    std::vector<Hole> holes;
    std::vector<Eigen::Vector3i> mIndices;
    std::vector<unsigned int> mValence;

    surface_mesh::Surface_mesh mHalfEdge;

    GLuint mIndicesBuffer;

    void fillHolesNaive(Hole h);
    bool isPlanar(Hole h);
    void divideComplexHoles(std::vector<Hole> complexHoles);


};


#endif // MESH_H
