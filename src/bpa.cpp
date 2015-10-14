#include "bpa.h"
using namespace std;
using namespace Eigen;

double distance(Vector3f p1, Vector3f p2){
    return(sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+
            (p1[1]-p2[1])*(p1[1]-p2[1])+
            (p1[2]-p2[2])*(p1[2]-p2[2])));
}

Vector3f getSphereCenter(Vector3f p1, Vector3f p2, Vector3f p3, double r){
    double a,b,c,as,bs,cs;
    a = distance(p2,p3);
    b = distance(p1,p3);
    c= distance(p1,p2);

    as = a*a;
    bs= b*b;
    cs = c*c;

    Vector3f bary(as*(bs+cs-as),bs*(as+cs-bs),cs*(as+bs-cs));
    double circumRadius = (as*bs*cs)/((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
    Vector3f n = p1.cross(p2);
    return (bary+(sqrt((r*r)-circumRadius))*n);
}


BPA::BPA(PointCloud model, Octree octree)
{
    double min1,min2;

    vector<Vector3f> positions = octree.getPositions();
    Vector3f p1,p2,p3;
    p1 = positions[0];
    p2 = positions[1];
    p3 = positions[2];
    min1  = distance(p1, p2);
    min2 = distance(p1, p3);
    if(min1 > min2){
        min1 = distance(p1, p3);
        min2 =  distance(p1,p2);
        p3 = positions[1];
        p2 = positions[2];
    }
    for(int i =3; i < 8; i++){
        if(distance(p1,positions[i]) < min1){
            p3 = p2;
            min2 = min1;
            p2 = positions[i];
            min1 = distance(p1,p2);

        }
        else if(distance(p1,positions[i]) < min2){
            p3 = positions[i];
            min2 = distance(p1,p3);
        }
    }
    double r = distance(p1,p3)/2;
    Sphere sph(r,getSphereCenter(p1,p2,p3,r));
}

