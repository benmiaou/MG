#ifndef VOLONOI_H
#define VOLONOI_H
#include <Eigen/Core>
#include <vector>
#include <Eigen/Dense>

class Tetrahedra
{
    Tetrahedra(Eigen::Vector3f p1,Eigen::Vector3f p2 , Eigen::Vector3f p3, Eigen::Vector3f p4)
        : a(p1),b(p2),c(p3),d(p4){}


public:
    Eigen::Vector3f a,b,c,d;
     std::vector<Tetrahedra> adjacentTetra;

friend class volonoi;
};

class volonoi
{
public:
    volonoi(std::vector<Eigen::Vector3f> points);

private:
    void majTetrahedron(Tetrahedra t);
    void insert(Eigen::Vector3f p);
     std::vector<Tetrahedra> tetrahedron;
};

#endif // VOLONOI_H
