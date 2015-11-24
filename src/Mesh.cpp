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
        mColors.push_back(Vector3f(0,0,0));
        surface_mesh::Normal normal = mHalfEdge.compute_vertex_normal(*vit).normalize();;

        mNormals.push_back(Vector3f(normal[0],normal[1],normal[2]));
        mNormals.push_back(Vector3f(vnormals[*vit][0],vnormals[*vit][1],vnormals[*vit][2]));
        mValence.push_back(mHalfEdge.valence(*vit));
    }


    int min = mValence[0];
    int max = mValence[0];
    int mean = 0;
    for(int i =1; i< mValence.size(); i++){
        int val = mValence[i];
        if(val > max)
            max = val;
        if(val < min )
            min = val;
        mean += val;
    }
    mColorVal.resize(mColors.size(),Vector3f(0,0,0));
        mColorHoles.resize(mColors.size(),Vector3f(0,0,0));

        for(int i =0; i< mValence.size(); i++){
            float val = mValence[i];
            float color = val/max;
            if(val == 0)
                mColorVal[i] = Vector3f(1,1,1);
            mColorVal[i] = Vector3f(color,color,color);
        }
    mean = mean/mValence.size();
    cout << "Valence: Max : " << max  << " Min : " << min << " Mean : " << mean <<endl;



    // face iterator
    Surface_mesh::Face_iterator fit, fend = mHalfEdge.faces_end();
    // vertex circulator
    Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    Surface_mesh::Vertex v0, v1, v2;
    int cpt;
    for (fit = mHalfEdge.faces_begin(); fit != fend; ++fit)
    {

        fvit = fvend = mHalfEdge.vertices(*fit);
        v0 = *fvit;
        ++fvit;
        v2 = *fvit;

        //do{
            v1 = v2;
            ++fvit;
            v2 = *fvit;
            mIndices.push_back(Vector3i(v0.idx(), v1.idx(), v2.idx()));

        //} while (++fvit != fvend);
            Surface_mesh::Edge e1 = mHalfEdge.find_edge(v0,v1);
            Surface_mesh::Edge e2 = mHalfEdge.find_edge(v0,v2);
            Surface_mesh::Edge e3 = mHalfEdge.find_edge(v1,v2);
            float le1 = mHalfEdge.edge_length(e1);
            float le2 = mHalfEdge.edge_length(e2);
            float le3 = mHalfEdge.edge_length(e3);

            float ratio = abs(le1/le2 - le1/le3);


    }
    for (int i=0; i<mPositions.size(); i++){
           if (i%2)
               mColorFaces.push_back(Vector3f(1,1,1));
           else
               mColorFaces.push_back(Vector3f(0,0,0));
       }
    find();
}

void Mesh::find(){
    float nbHoles = 0;
    Surface_mesh::Halfedge_iterator hit,hend = mHalfEdge.halfedges_end();
    vector<bool> alreadyDo;
    alreadyDo.resize(mHalfEdge.halfedges_size());
    int cpt = 0;
    for(hit = mHalfEdge.halfedges_begin();hit != hend; ++hit){
        Surface_mesh::Halfedge current = *hit;
        Surface_mesh::Face f = mHalfEdge.face(*hit);
        if(!f.is_valid() /*&& !alreadyDo[current.idx()]*/){

            cpt = 0;
            Surface_mesh::Vertex v = mHalfEdge.vertex(mHalfEdge.edge(*hit),0);

            Surface_mesh::Halfedge next = mHalfEdge.next_halfedge(*hit);
            f = mHalfEdge.face(next);
            while(!f.is_valid() && next != *hit){
                next = mHalfEdge.next_halfedge(next);
                f = mHalfEdge.face(next);
                cpt++;
                 alreadyDo[next.idx()] = true;
            }

            int cptt = cpt;
            cpt =0;
            Surface_mesh::Halfedge prev = mHalfEdge.prev_halfedge(*hit);
            f = mHalfEdge.face(prev);
            while(!f.is_valid() && next != *hit){
                prev = mHalfEdge.prev_halfedge(next);
                f = mHalfEdge.face(prev);
                cpt++;
                 alreadyDo[prev.idx()] = true;
            }

            if (cptt > cpt)
                    cpt = cptt;
            if(cpt > 3)
                mColorHoles[v.idx()] += Vector3f(1,0,0);
            if(cpt < 3)
                mColorHoles[v.idx()] += Vector3f(0,0,1);
            if(cpt == 3)
                mColorHoles[v.idx()] += Vector3f(0,1,0);
            nbHoles += 1.0/cpt;
        }


    }
      cout << "nbHoles : " << (int)nbHoles <<endl;
}


void Mesh::init(Shader *shader)
{
    //ajout
    PointCloud::init(shader);

    glGenVertexArrays(1, &mVao);


    glBindVertexArray(mVao);



    glGenBuffers(1, &mIndicesBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Vector3i) *mIndices.size(), mIndices.data(),  GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mPositions.size(), mPositions.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mNormals.size(), mNormals.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mColors.size(), mColors.data(), GL_STATIC_DRAW);

    specifyVertexData(shader);

    glBindVertexArray(0);

    mReady = true;
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


