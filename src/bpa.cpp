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

int BPA::getSeed(Octree *octree ,vector<bool> &isVisited,  vector<Vector3f> &positions,vector<int> &ids, double &r){
    double rc = r*2;
    double min1,min2;
    int idx0, idx1;
    Vector3f p1;
    int seed = 0;

    ids.clear();
    ids.resize(3);

    while(isVisited[seed]){
        seed++;
        if(seed >= isVisited.size())
            return -1;
    }
    isVisited[seed] = true;
    p1 = positions[seed];
    ids[0] = seed;
    octree->getNeighbour(p1,0,idx0,idx1);

    min1  = -1;
    min2 = -1;
    for(int i =idx0+3; i < idx1; i++){
        if(!isVisited[i])
            if(distance(p1, positions[i]) <= rc){
                if(min1 == -1 || distance(p1,positions[i]) < min1){
                    ids[2] = ids[1];
                    min2 = min1;
                    ids[1] = i;
                    min1 = distance(p1,positions[i]);

                }
                else if(min2 == -1 || distance(p1,positions[i]) < min2){
                    ids[2] = i;
                    min2 = distance(p1,positions[i]);
                }
            }
    }
    if(min1 != -1 && min2 != -1)
        return 1;
    else
        return 0;
}


BPA::BPA(PointCloud *model, Octree *octree)
{

    int idx0, idx1;
    Vector3f p1,p2,p3,n1,n2,n3;


    vector<Vector3f> positions =  octree->getPositions();
    vector<Vector3f> normals =  octree->getNormals();
    vector<bool> isVisited(positions.size(),false);
    vector<int> ids;

    mVertices = positions;
    mNormals = normals;
    int cpt = 0;
    double r = 0.05;



    while(true){
        int result = getSeed(octree,isVisited,positions,ids,r);
        if(result == -1)
            break;
        if(result == 1){
            isVisited[ids[1]] = true;
            isVisited[ids[2]] = true;

            p1 = positions[ids[0]];
            p2 = positions[ids[1]];
            p3 = positions[ids[2]];

            n1 = normals[ids[0]];
            n2 = normals[ids[1]];
            n3 = normals[ids[2]];

            Vector3f acN = (n1+n2+n3)/3;
            Vector3f center = getSphereCenter(p1,p2,p3,r,acN);
            //std::cout <<"SEED : \nP1 = "<< p1 << "\nP2 "<<p2 << "\nP3 "<< p3 <<std::endl;
            actualSphere = new BPASphere(r,center);
            mIndices.push_back(Vector3i(ids[0],ids[1],ids[2]));

            //sauvegarde du 1er triangle et push de ses aretes
            Edge edge12(ids[0],ids[1],actualSphere->center);
            Edge edge13(ids[0],ids[2],actualSphere->center);
            Edge edge23(ids[1],ids[2],actualSphere->center);
            edges.push_back(edge12);
            edges.push_back(edge13);
            edges.push_back(edge23);



            while(!edges.empty() ){
                Edge edge = edges.back();
                edges.pop_back();
                actualSphere->center = edge.sphereCenter;
                p1 = positions[edge.id1];
                p2 = positions[edge.id2];
                int id = -1;
                for(int i = 0; i < 20 && id ==-1 ; i++){
                    moveSphere(0.1*M_PI,(p1-p2));
                    octree->getNeighbour(actualSphere->center,0,idx0,idx1);
                    for(int i =idx0; i < idx1; i++){
                        //if(!isVisited[i]){
                        Vector3f actualP = positions[i];
                        //if(distance(edge.sphereCenter, actualP) >= r)
                        if(distance(actualSphere->center, actualP) <= r){//dans la nouvelle sphere
                            if(distance(p1, actualP) > 0 && distance(p2, actualP) > 0)
                                if(id == -1 || distance(actualSphere->center, actualP) > distance(actualSphere->center,positions[id]))
                                    id = i;
                            // }
                        }
                    }
                }

                if(id != -1){
                    if(!isVisited[id]){
                        isVisited[id] = true;
                        p3 = positions[id];
                        n1 = normals[edge.id1];
                        n2 = normals[edge.id2];
                        n3 = normals[id];
                        Vector3f acN = (n1+n2+n3)/3;
                        actualSphere->center = getSphereCenter(p1,p2,p3,r,acN);

                        Edge edge13(edge.id1,id,actualSphere->center);
                        Edge edge23(edge.id2,id,actualSphere->center);
                        std::vector<Edge>::iterator it;
                        it = edges.begin();
                        edges.insert(it,edge13);
                        it = edges.begin();
                        edges.insert(it,edge23);
                    }
                    /*
                    edges.push_back(edge13);
                    edges.push_back(edge23);
*/

                    mIndices.push_back(Vector3i(edge.id1,edge.id2,id));
                }
            }

        }
        cpt++;
    }
}



void BPA::init(Shader *shader)
{
    glGenVertexArrays(1, &mVao);
    glGenBuffers(3, mBufs);

    glBindVertexArray(mVao);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBufs[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Vector3i) *mIndices.size(), mIndices.data(),  GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mVertices.size(), mVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mBufs[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*mNormals.size(), mNormals.data(), GL_STATIC_DRAW);

    specifyVertexData(shader);

    glBindVertexArray(0);

    mReady = true;
}

void  BPA::specifyVertexData(Shader *shader)
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



void  BPA::draw(Shader *shader, bool drawEdges)
{
    if (!mReady) {
        std::cerr<<"Warning: Sphere not ready for rendering" << std::endl;
        return;
    }
    glBindVertexArray(mVao);

    glDrawElements(drawEdges ? GL_LINE_LOOP : GL_TRIANGLES, sizeof(Vector3i) *mIndices.size(),  GL_UNSIGNED_INT, 0);
    //glDrawArrays(GL_POINTS, 0, mVertices.size());

    glBindVertexArray(0);
}

void BPA::moveSphere(double theta, Vector3f axis){
    axis.normalized();
    Matrix3f mat;
    mat << 1, 0, 0,
            0, 1, 0,
            0, 0, 1 ;
    mat = Matrix3f(AngleAxisf(theta, axis));
    Affine3f object_matrix;
    object_matrix = Translation3f(actualSphere->center) * mat * (Translation3f(-1*actualSphere->center));
    actualSphere->center = object_matrix * actualSphere->center;
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
