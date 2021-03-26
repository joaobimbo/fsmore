#include <fsmore/map_fsmore.h>

MapFsmore::MapFsmore(){
    map = new octomap::OcTree(0.01);
}


Line MapFsmore::ForceToLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Vector3f &p1, Eigen::Vector3f &p2,float &k){

    // https://physics.stackexchange.com/questions/98633/line-of-action-force
    // r = (F x M) / (F . F)
    // h = (F . M) / (F . F)

    Eigen::Vector3f r=(F.cross(M)/F.dot(F));
    k=F.dot(M)/F.dot(F);

    Eigen::Vector3f Fn=F.normalized();
    p1.x()=r.x()-line_half_length*Fn.x();
    p1.y()=r.y()-line_half_length*Fn.y();
    p1.z()=r.z()-line_half_length*Fn.z();

    p2.x()=r.x()+line_half_length*Fn.x();
    p2.y()=r.y()+line_half_length*Fn.y();
    p2.z()=r.z()+line_half_length*Fn.z();

    Line l(p1,p2);

    return(l);
}


bool MapFsmore::AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T){
    Eigen::Vector3f p1_1,p1_2;
    float k;
    Line l_o=ForceToLine(F,M,p1_1,p1_2,k);
    octomap::point3d p1(l_o.p1.x(),l_o.p1.y(),l_o.p1.z());
    octomap::point3d p2(l_o.p2.x(),l_o.p2.y(),l_o.p2.z());
    map->insertRay(p1,p2);

    return(true);
}

template <class PType> std::vector<PType> MapFsmore::getPointCloud(octomap::OcTree *octree){
    std::vector<PType> pc;
    for(octomap::OcTree::iterator it = octree->begin();
        it!=octree->end();it++){
        PType p;
        p.x=it.getX();
        p.y=it.getY();
        p.z=it.getZ();
        pc.push_back(p);
    }
    return(pc);
}


template <class PType> std::vector<PType> MapFsmore::getMapPointCloud(){
  //DEBUG
  std::vector<PType> pc;
  pc=getPointCloud<PType>(map);
  return(pc);
}

