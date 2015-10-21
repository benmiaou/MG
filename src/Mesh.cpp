#include "Mesh.h"
#include "Meshloader.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace surface_mesh;

Mesh::~Mesh()
{
    if(mReady){
        glDeleteBuffers(1, &mIndicesBuffer);
        glDeleteVertexArrays(1,&mVao);
    }
}

void Mesh::load(const string& filename)
{
    cout << "Loading: " << filename << endl;

    mHalfEdge.read(filename);
    mHalfEdge.update_face_normals();
    mHalfEdge.update_vertex_normals();

    // vertex properties
    Surface_mesh::Vertex_property<Point> vertices = mHalfEdge.get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point> vnormals = mHalfEdge.get_vertex_property<Point>("v:normal");


    // vertex iterator
    Surface_mesh::Vertex_iterator vit;

    for(vit = mHalfEdge.vertices_begin(); vit != mHalfEdge.vertices_end(); ++vit)
    {
        mPositions.push_back(Vector3f(vertices[*vit][0],vertices[*vit][1],vertices[*vit][2]));
        //surface_mesh::Normal normal = mHalfEdge.compute_vertex_normal(*vit).normalize();;

        //mNormals.push_back(Vector3f(normal[0],normal[1],normal[2]));
        mNormals.push_back(Vector3f(vnormals[*vit][0],vnormals[*vit][1],vnormals[*vit][2]));
        mValence.push_back(mHalfEdge.valence(*vit));
    }
    // face iterator
    Surface_mesh::Face_iterator fit, fend = mHalfEdge.faces_end();
    // vertex circulator
    Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    Surface_mesh::Vertex v0, v1, v2;

    for (fit = mHalfEdge.faces_begin(); fit != fend; ++fit)
    {

        fvit = fvend = mHalfEdge.vertices(*fit);
        v0 = *fvit;
        ++fvit;
        v2 = *fvit;

        do{
            v1 = v2;
            ++fvit;
            v2 = *fvit;
            mIndices.push_back(Vector3i(v0.idx(), v1.idx(), v2.idx()));
        } while (++fvit != fvend);
    }
}

void Mesh::find(){
    Surface_mesh::Halfedge_iterator hit,hend = mHalfEdge.halfedges_end();

    for(hit = mHalfEdge.halfedges_begin();hit != hend; ++hit){
        Surface_mesh::Face f = mHalfEdge.face(*hit);
        if(!f.is_valid()){

        }
    }
}


void Mesh::init(Shader *shader)
{
    glGenVertexArrays(1, &mVao);
    glGenBuffers(3, mBufs);

    glBindVertexArray(mVao);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBufs[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Vector3i) *mIndices.size(), mIndices.data(),  GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mPositions.size(), mPositions.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mNormals.size(), mNormals.data(), GL_STATIC_DRAW);

    specifyVertexData(shader);

    glBindVertexArray(0);



    mReady = true;
}

void Mesh::specifyVertexData(Shader *shader)
{
    mShader = shader;

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[1]);
    int pos_loc = shader->getAttribLocation("vtx_position");
    glEnableVertexAttribArray(pos_loc);
    glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3f), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[2]);
    int normal_loc = shader->getAttribLocation("vtx_normal");
    if(normal_loc>=0){
        glEnableVertexAttribArray(normal_loc);
        glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3f), (void*)0);
    }
}


void Mesh::draw(Shader *shader, bool drawEdges) 
{
    if (!mReady) {
        cerr<<"Warning: Mesh not ready for rendering" << endl;
        return;
    }

    glBindVertexArray(mVao);

    if(mShader->id() != shader->id()){
        specifyVertexData(shader);
    }

    glDrawElements(drawEdges ? GL_LINE_LOOP : GL_TRIANGLES, mIndices.size()*sizeof(Vector3i),  GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
}


