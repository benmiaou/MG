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
    //std::cout <<"\nn: "<< n << "\n acN " <<acN <<"\ndot " << acN.dot(n)<<std::endl;
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
    int idx0, idx1;
    Vector3f p1,p2,p3,n1,n2,n3;

    vector<Vector3f> positions =  octree->getPositions();
    vector<Vector3f> normals =  octree->getNormals();

    p1 = positions[10];
    n1 = normals[10];
    octree->getNeighbour(p1, idx0,idx1);
    std::cout <<"\n0: "<< idx0 << "\n1 " <<idx1 <<std::endl;


    p1 = positions[idx0];
    n1 = normals[idx0];


    p2 = positions[idx0+1];
    p3 = positions[idx0+2];

    n2 = normals[idx0+1];
    n3 = normals[idx0+2];

    min1  = distance(p1, p2);
    min2 = distance(p1, p3);
    if(min1 > min2){
        min1 = distance(p1, p3);
        min2 =  distance(p1,p2);
        p3 = positions[idx0+1];
        p2 = positions[idx0+2];
        n3 = normals[idx0+1];
        n2 = normals[idx0+2];
    }
    for(int i =idx0+3; i < idx1; i++){
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
    std::cout <<"P1 = "<< p1 << "\nP2 "<<p2 << "\nP3 "<< p3 <<std::endl;
    actualSphere = new BPASphere(r,center);
    mVertices.push_back(p1);
    mVertices.push_back(p2);
    mVertices.push_back(p3);
    mIndices.push_back(0);
    mIndices.push_back(1);
    mIndices.push_back(2);

    //sauvegarde du 1er triangle et push de ses aretes
        this->actualTriangle = new BPATriangle(p1, p2, p3);
        Vector3f edge12(p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]);
        Vector3f edge13(p1[0]-p3[0], p1[1]-p3[1], p1[2]-p3[2]);
        Vector3f edge23(p2[0]-p3[0], p2[1]-p3[1], p2[2]-p3[2]);
        edges.push_back(edge12);
        edges.push_back(edge13);
        edges.push_back(edge23);


    /*while(mIndices.size() < 2000){
        std::cout <<"T1"<<std::endl;
        Vector3f tmp;
        p3 = positions[findNext(p1,p2,actualSphere,octree)];
        std::cout <<"T2"<<std::endl;
        mVertices.push_back(p3);
        std::cout <<"T3"<<std::endl;
        mIndices.push_back(mVertices.size()-4);
        mIndices.push_back(mVertices.size()-3);
        mIndices.push_back(mVertices.size()-1);
        acN = (n1+n2+n3)/3;
        center = getSphereCenter(p1,p2,p3,r,acN);
        actualSphere->center = center;
        p1 = p2;
        p2 = p3;
         std::cout <<"P1 = "<< p1 << "\nP2 "<<p2 << "\nP3 "<< p3 <<std::endl;*/


    }
}

int BPA::findNext(Vector3f p1, Vector3f p2, BPASphere *actualSphere, Octree *octree){
    int id = 0;
    vector<Vector3f> positions =  octree->getPositions();
    vector<Vector3f> normals =  octree->getNormals();
    Vector3f m = (p1+p2)/2;


    Vector3f center = actualSphere->center;
    int r = actualSphere->radius;
    float rp = distance(m,center) + r;
    int idx0, idx1;
    octree->getNeighbour(m, idx0,idx1);
    std::cout <<"\n0: "<< idx0 << "\n1 " <<idx1 <<std::endl;
    Vector3f p3 = positions[idx0];
    id = idx0;
    for(int i =idx0+1; i < idx1; i++){
        Vector3f actualP = positions[i] ;
        if(distance(center, actualP) > r){
                //&& distance(m, actualP) < rp){
               // && distance(p1, actualP ) >0
               // && distance(p2, actualP ) >0){
               // && distance(p3, m ) >  distance(m, actualP)){
            p3 = positions[i];
            id = i;
             std::cout <<"TTIF"<<std::endl;
        }

    }
    std::cout <<"TT1"<<std::endl;
    return id;
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
    //glDrawArrays(GL_POINTS, 0, mVertices.size());

    glBindVertexArray(0);
}

void BPA::moveSphere(double theta, Vector3f axis){
  Matrix3f mat;
  mat << 1, 0, 0,
         0, 1, 0,
         0, 0, 1 ;
  mat = Matrix3f(AngleAxisf(theta, axis));
  actualSphere->center = mat * this->getCenter();  
}

Vector3f BPA::findPoint(vector<Vector3f> tab){
  Vector3f center = this->getCenter();
  double rad =  this->getRadius();
  for (int i=0; i<tab.size(); i++){
    double x = (tab[i][0] - center[0])*(tab[i][0] - center[0]);
    double y = (tab[i][1] - center[1])*(tab[i][1] - center[1]);
    double z = (tab[i][2] - center[2])*(tab[i][2] - center[2]);
    if (x+y+z==rad*rad)
      return Vector3f(tab[i][0], tab[i][1], tab[i][2]);
  }
  return Vector3f(-1, -1, -1);
}

void BPA::findNewTriangle(double theta, Vector3f axis){
  //while (){
    moveSphere(theta, axis);
    Vector3f point = findPoint(this->mVertices); //3eme point du nouveau triangle
    
    //}
}
