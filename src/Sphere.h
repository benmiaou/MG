#ifndef _SPHERE_H
#define _SPHERE_H

#include "Shape.h"
#include <vector>

class Sphere :public Shape {

public:
    Sphere(float radius=1.f, int nU=40, int nV=40);
    ~Sphere();




    void init(Shader *shader);
    void draw(Shader *shader, bool drawEdges = false);
    float radius() const { return mRadius; }

private :
    void specifyVertexData(Shader *shader);
    GLuint mVao;
    GLuint mBufs[2];

    std::vector<int>        mIndices;   /** vertex indices */
    std::vector<Eigen::Vector3f>	mVertices;  /** 3D positions */
    std::vector<Eigen::Vector3f>	mColors;    /** colors */


    float mRadius;
};

#endif
