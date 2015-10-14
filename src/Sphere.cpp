#include "Sphere.h"

using namespace Eigen;

Sphere::Sphere(float radius, int nU, int nV) :
    mRadius(radius)
{
    int nVertices  = (nU + 1) * (nV + 1);
    int nTriangles =  nU * nV * 2;

    mVertices.resize(nVertices);
    mIndices.resize(3*nTriangles);
std::cout <<"r = " << mRadius <<std::endl;
    for(int v=0;v<=nV;++v)
    {
        for(int u=0;u<=nU;++u)
        {
            Vector3f vertex ,normal;
            float theta = u / float(nU) * M_PI;
            float phi 	= v / float(nV) * M_PI * 2;
            int index 	= u +(nU+1)*v;
            // normal
            normal[0] = sin(theta) * cos(phi);
            normal[1] = sin(theta) * sin(phi);
            normal[2] = cos(theta);
            normal.normalize();

            vertex = normal * mRadius;

            mVertices[index] = vertex;
        }
    }

    int index = 0;
    for(int v=0;v<nV;++v)
    {
        for(int u=0;u<nU;++u)
        {
            int vindex 	= u + (nU+1)*v;

            mIndices[index+0] = vindex;
            mIndices[index+1] = vindex+1 ;
            mIndices[index+2] = vindex+1 + (nU+1);

            mIndices[index+3] = vindex;
            mIndices[index+4] = vindex+1 + (nU+1);
            mIndices[index+5] = vindex   + (nU+1);

            index += 6;
        }
    }
}


Sphere::~Sphere()
{
    if(mReady)
    {
        glDeleteBuffers(1, mBufs);
    }
}

void Sphere::init(Shader *shader)
{

    glGenVertexArrays(1, &mVao);
    glGenBuffers(2, mBufs);

    glBindVertexArray(mVao);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBufs[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) *mIndices.size(), mIndices.data(),  GL_STATIC_DRAW);

    int vertex_loc = shader->getAttribLocation("vtx_position");
    glEnableVertexAttribArray(vertex_loc);
    glBindBuffer(GL_ARRAY_BUFFER, mBufs[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f) * mVertices.size(), mVertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(vertex_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);




    specifyVertexData(shader);

    glBindVertexArray(0);

    mReady = true;
}

void Sphere::specifyVertexData(Shader *shader)
{
    mShader = shader;


}



void Sphere::draw(Shader *shader, bool drawEdges)
{
    if (!mReady) {
        std::cerr<<"Warning: Sphere not ready for rendering" << std::endl;
        return;
    }
    glBindVertexArray(mVao);

    glDrawElements(GL_TRIANGLES,mIndices.size(),GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
}
