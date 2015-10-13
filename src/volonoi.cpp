#include "volonoi.h"


using namespace Eigen;
int inSphere(Vector3f a,Vector3f b,Vector3f c,Vector3f d,Vector3f p){
    std::vector<Vector3f> points{a,b,c,d,p};
    MatrixXf mat(5,5);
    for(int i =0; i< 5; i++){
        Vector3f point = points[i];
        for(int j =0; j <3; j++){
            mat(i,j) = point[j];
        }
         mat(i,3) = point[0]*point[0]+point[1]*point[1]+point[2]*point[2];
         mat(i,4) = 1;
    }
    int det = mat.determinant();
    if(det > 0)
        return 1;
    else if(det < 0)
        return -1;
    return 0;
}

int orient(Vector3f a,Vector3f b,Vector3f c,Vector3f p){
    std::vector<Vector3f> points{a,b,c,p};
    MatrixXf mat(4,4);
    for(int i =0; i< 4; i++){
        Vector3f point = points[i];
        for(int j =0; j <3; j++){
            mat(i,j) = point[j];
        }
         mat(i,3) = 1;
    }
    int det = mat.determinant();
    if(det > 0)
        return 1;
    else if(det < 0)
        return -1;
    return 0;
}

Tetrahedra findAdj(Tetrahedra t, Vector3f a,Vector3f b,Vector3f c){
    for(int i = 0; i<t.adjacentTetra.size();i++){
        Tetrahedra adj = t.adjacentTetra[i];
        if(adj.a == a && adj.b == b && adj.c == c)
            return adj;
    }

}
void volonoi::majTetrahedron(Tetrahedra t){
   /* Tetrahedra t1(t.a,t.b,t.c,p);
    Tetrahedra t2(t.d,t.a,t.b,p);
    Tetrahedra t3(t.c,t.d,t.a,p);
    Tetrahedra t4(t.b,t.c,t.d,p);*/
    //tetrahedron.erase(find(tetrahedron.begin(), tetrahedron.end()));


    /*tetrahedron.push_back(t1);
    tetrahedron.push_back(t2);
    tetrahedron.push_back(t3);
    tetrahedron.push_back(t4);*/
}

void volonoi::insert(Vector3f p){
   // Tetrahedra t = findTetraContainingP(p);


    //majTetrahedron(t);



   /* for(int i = 0; i < 4; i++){
        t = tetrahedron[i];
        Tetrahedra ta = findAdj(t,t[0],t[1],t[2]);
        if(inSphere(t[0],t[1],t[2],t[3],ta[3]) == 1){

        }

    }*/
}



volonoi::volonoi(std::vector<Eigen::Vector3f> points)
{

}


