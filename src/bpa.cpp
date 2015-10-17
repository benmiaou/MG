#include "bpa.h"
using namespace std;
using namespace Eigen;

double distance(Vector3f p1, Vector3f p2){
    return(sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+
            (p1[1]-p2[1])*(p1[1]-p2[1])+
            (p1[2]-p2[2])*(p1[2]-p2[2])));
}

double BPA::getRadius(){
    return actualSphere->radius;
}

Eigen::Vector3f BPA::getCenter(){
    return actualSphere->center;
}



Vector3f getSphereCenter(Vector3f p1, Vector3f p2, Vector3f p3, double r,Vector3f acN){
    double a,b,c,as,bs,cs;
    a = distance(p2,p3);
    b = distance(p1,p3);
    c = distance(p1,p2);

    as = a*a;
    bs= b*b;
    cs = c*c;

    Vector3f circumcenter(as*(bs+cs-as),bs*(as+cs-bs),cs*(as+bs-cs));
    double circumRadius = (as*bs*cs)/((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
    Vector3f n = p1.cross(p2);
    n.normalize();
    std::cout <<"\nn: "<< n << "\n acN " <<acN <<"\ndot " << acN.dot(n)<<std::endl;
    if(acN.dot(n) < 0)
        n = n*-1;
    //std::cout <<"\ncircumcenter Normalized : "<< circumcenter <<std::endl;
    circumcenter = (circumcenter[0]*p1
            + circumcenter[1]*p2 + circumcenter[2]*p3)
            /(circumcenter[0]+circumcenter[1]+circumcenter[2]);
    //std::cout <<"\nn = "<< n<< "\nr = " << r << "\nRadius = "<< circumRadius << "\nTruc = "<< (sqrt((r*r)-circumRadius)) << "\ncircumcenter"<< circumcenter <<std::endl;
    Vector3f center = circumcenter+(sqrt((r*r)-circumRadius))*n;
    //std::cout <<"Pos = "<< center << "\np1 "<<p1 <<std::endl;
    return center;
}


BPA::BPA(PointCloud *model, Octree *octree)
{
    double min1,min2;

    vector<Vector3f> positions = model->getPositions();
    vector<Vector3f> normals = model->getNormals();
    Vector3f p1,p2,p3,n1,n2,n3;
    p1 = positions[0];
    p2 = positions[1];
    p3 = positions[2];

    n1 = normals[0];
    n2 = normals[1];
    n3 = normals[2];

    min1  = distance(p1, p2);
    min2 = distance(p1, p3);
    if(min1 > min2){
        min1 = distance(p1, p3);
        min2 =  distance(p1,p2);
        p3 = positions[1];
        p2 = positions[2];
        n3 = normals[2];
        n2 = normals[1];
    }
    for(int i =3; i < positions.size(); i++){
        if(distance(p1,positions[i]) < min1){
            p3 = p2;
            n3 = n2;
            min2 = min1;
            p2 = positions[i];
            n2 = normals[i];
            min1 = distance(p1,p2);

        }
        else if(distance(p1,positions[i]) < min2){
            p3 = positions[i];
            n3 = normals[i];
            min2 = distance(p1,p3);
        }
    }
    double r = min2;
    Vector3f acN = (n1+n2+n3)/3;
    Vector3f center = getSphereCenter(p1,p2,p3,r,acN);
    actualSphere = new BPASphere(r,center);
    mVertices.push_back(p1);
    mVertices.push_back(p2);
    mVertices.push_back(p3);
    mIndices.push_back(0);
    mIndices.push_back(1);
    mIndices.push_back(2);
}


void BPA::init(Shader *shader)
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

void  BPA::specifyVertexData(Shader *shader)
{
    mShader = shader;


}



void  BPA::draw(Shader *shader, bool drawEdges)
{
    if (!mReady) {
        std::cerr<<"Warning: Sphere not ready for rendering" << std::endl;
        return;
    }
    glBindVertexArray(mVao);

    glDrawElements(GL_TRIANGLES,mIndices.size(),GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
}
