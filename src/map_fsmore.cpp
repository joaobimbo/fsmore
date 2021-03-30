#include <fsmore/map_fsmore.h>

MapFsmore::MapFsmore()
    : pc_obj(new PCType),
      pc_map(new PCType){
    oct_map = OctType::Ptr(new OctType(0.01));
    oct_obj = OctType::Ptr(new OctType(0.01));

    oct_map->setInputCloud(pc_map);
    oct_map->addPointsFromInputCloud();

    oct_obj->setInputCloud(pc_obj);
    oct_obj->addPointsFromInputCloud();


}

std::size_t MapFsmore::KeyHasher(MapKey key_arg){
    std::size_t seed = 0;
    boost::hash_combine(seed, key_arg.x);
    boost::hash_combine(seed, key_arg.y);
    boost::hash_combine(seed, key_arg.z);
    return seed;

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
    Eigen::Vector3f dist=(l_m.p2-l_m.p1);
    int n_points=(int) dist.norm()/line_res;
    for (int i=0;i<n_points;i++){
        PType p;
        Eigen::Vector3f p2=l_m.p1+((float) i*line_res*l_m.dir.normalized());
        p.x=p2.x();
        p.y=p2.y();
        p.z=p2.z();
        ray.push_back(p);
    }
    //oct_map->getIntersectedVoxelCenters(l_m.p1,l_m.p2,ray);

    lines_map.push_back(l_m);
    lines_obj.push_back(l_o);


    for (PCType::iterator it=ray.begin();it!=ray.end();it++){
        MapKey key_map,key_obj;
        Voxel *v_m,*v_o;

        //Do Map things
        if(!oct_map->isVoxelOccupiedAtPoint(*it) || map_map.find(KeyHasher(oct_map->GetKeyAtPoint(*it)))==map_map.end()){
            PType p= *it;
            p.intensity=1.0f/ray.size();
            oct_map->addPointToCloud(p,pc_map);
            key_map=oct_map->GetKeyAtPoint(*it);
            v_m=new Voxel(oct_map,p,KeyHasher(key_map));
            map_map.insert(std::pair<size_t,Voxel>(KeyHasher(key_map),*v_m));
}
        else{
            //oct_map->findLeafAtPoint(*it)->
            key_map=oct_map->GetKeyAtPoint(*it);
            //if(map_map.find(KeyHasher(key))!=map_map.end()){
            v_m=&(map_map.at(KeyHasher(key_map)));
            //}
        }
        v_m->intersecting.push_back(&lines_map.back());

        //Do object things
        PType p_o = v_m->TransformCoord(T.inverse());

        if(!oct_obj->isVoxelOccupiedAtPoint(p_o)){// ||  map_obj.find(KeyHasher(oct_obj->GetKeyAtPoint(p_o)))==map_obj.end()){
           /* p_o.intensity=0.0;
            oct_obj->addPointToCloud(p_o,pc_obj);
            key_obj=oct_obj->GetKeyAtPoint(p_o);
            v_o=new Voxel(oct_obj,p_o,KeyHasher(key_obj));
            //v_o->likelihood=p_o.intensity;
            map_obj.insert(std::pair<size_t,Voxel>(KeyHasher(key_obj),*v_o));*/
        }
        else{
            key_obj=oct_obj->GetKeyAtPoint(p_o);
            v_o=&(map_obj.at(KeyHasher(key_obj)));
            //v_o->likelihood=std::max(v_o->likelihood,v_m->likelihood);
            *(v_m->likelihood)=*std::max(v_o->likelihood,v_m->likelihood);

            v_o->intersecting.push_back(&lines_obj.back());
        }
    }

    return(true);
}

pcl::PointCloud<pcl::PointXYZI>  MapFsmore::getPointCloud(OctType::Ptr octree){
    pcl::PointCloud<pcl::PointXYZI>  pc;

    for(OctType::Iterator it = octree->leaf_depth_begin();it!=octree->leaf_depth_end();it++){
        pcl::PointXYZI p;
        pcl::octree::OctreeLeafNode<PType> n;

    }
    return(pc);
}


PCType MapFsmore::getMapPointCloud(){
    //pcl::PointCloud<pcl::PointXYZI>  pc;
    //pc=getPointCloud(oct_map);
    //return(pc);
    return(*pc_map);
}

PCType MapFsmore::getObjectPointCloud(){
    //pcl::PointCloud<pcl::PointXYZI>  pc;
    //pc=getPointCloud(oct_obj);
    //return(pc);
    return(*pc_obj);
}
