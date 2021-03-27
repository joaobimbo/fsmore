#include <fsmore/map_fsmore.h>

MapFsmore::MapFsmore()
    : pc_obj(new PCType),
      pc_map(new PCType){
    oct_map = OctType::Ptr(new OctType(0.01));
    oct_obj = OctType::Ptr(new OctType(0.01));

    oct_map->setInputCloud(pc_map);
    oct_obj->setInputCloud(pc_obj);

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
    Line l_m=l_o.Transform(T);

    PCType::VectorType ray;
    oct_map->getApproxIntersectedVoxelCentersBySegment(l_m.p1,l_m.p2,ray);

    lines_map.push_back(l_m);


    for (PCType::iterator it=ray.begin();it!=ray.end();it++){
        //size_t hash=hasher(*it);
        pcl::octree::OctreeKey key;


        oct_map->genOctreeKeyforPoint(*it,key);
        Voxel *v;
        if(map_map.find(key)==map_map.end()){
            v=new Voxel(oct_map,key);
            PType p= *it;
            p.intensity=1.0f/ray.size();
            oct_map->addPointToCloud(p,pc_map);
            map_map.insert(std::pair<pcl::octree::OctreeKey,Voxel>(key,*v));
        }
        else{
            v=&(map_map.at(key));
        }
        v->intersecting.push_back(&lines_map.back());
        std::vector<int> points_inside;
        oct_obj->voxelSearch(v->TransformCoord(T.inverse()),points_inside);


        v->likelihood=oct_obj->getInputCloud()->at(points_inside[0]).intensity*v->likelihood;
    }




    return(true);
}

pcl::PointCloud<pcl::PointXYZI>  MapFsmore::getPointCloud(OctType::Ptr octree){
    pcl::PointCloud<pcl::PointXYZI>  pc;

    for(OctType::Iterator it = octree->leaf_depth_begin();it!=octree->leaf_depth_end();it++){
        pcl::PointXYZI p;
        pcl::octree::OctreeLeafNode<PType> n;
        it.getLeafContainer().
                n.getContainer().x;


        p.x=it.getX();
        p.y=it.getY();
        p.z=it.getZ();
        p.intensity=it->getOccupancy();
        pc.push_back(p);
    }
    return(pc);
}


pcl::PointCloud<pcl::PointXYZI>  MapFsmore::getMapPointCloud(){
    pcl::PointCloud<pcl::PointXYZI>  pc;
    pc=getPointCloud(oct_map);
    return(pc);
}

pcl::PointCloud<pcl::PointXYZI>  MapFsmore::getObjectPointCloud(){
    pcl::PointCloud<pcl::PointXYZI>  pc;
    pc=getPointCloud(oct_obj);
    return(pc);
}
