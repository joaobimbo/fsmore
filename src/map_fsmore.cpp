#include <fsmore/map_fsmore.h>

MapFsmore::MapFsmore()
    : pc_obj(new PCType),
      pc_map(new PCType){
    //oct_map = OctType::Ptr(new OctType(0.01));
    //oct_obj = OctType::Ptr(new OctType(0.01));
    //oct_map = OctType::Ptr(new OctType());
    //oct_obj = OctType::Ptr(new OctType());
    oct_map = OctTypePtr(new OctType(line_res));
    oct_obj = OctTypePtr(new OctType(line_res));


    lines_map.reserve(1000);
    lines_obj.reserve(1000);

    //oct_map->setLeafSize(line_res,line_res,line_res);
    //oct_map->setInputCloud(pc_map);
    //oct_map->setSaveLeafLayout(true);
    //oct_map->addPointsFromInputCloud();

    //oct_obj->setLeafSize(line_res,line_res,line_res);
    //oct_obj->setInputCloud(pc_obj);
    //oct_obj->setSaveLeafLayout(true);
    //oct_obj->addPointsFromInputCloud();


}

std::size_t MapFsmore::KeyHasher(KeyType key_arg){
    std::size_t seed = 0;
    //boost::hash_combine(seed, key_arg.x());
    //boost::hash_combine(seed, key_arg.y());
    //boost::hash_combine(seed, key_arg.z());
    boost::hash_combine(seed, key_arg.k[0]);
    boost::hash_combine(seed, key_arg.k[1]);
    boost::hash_combine(seed, key_arg.k[2]);
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

bool MapFsmore::LineExists(std::vector<Line> &lines,Line &l_in){
    for (size_t i=0;i<lines.size();i++){
        float dp=l_in.InnerProd(lines.at(i));
        if(dp>same_line_tol) {
            Eigen::Vector3f dir2=lines.at(i).p2-l_in.p1;
            Eigen::Vector3f dir2b=lines.at(i).p1-l_in.p1;
            if(dir2b.norm()>dir2.norm()) dir2=dir2b; // find the farthest point to draw vector
            if(fabs(l_in.dir.dot(dir2))>same_line_tol){
                time(&lines.at(i).timestamp);
                return(true);
            }
        }
    }
    time(&l_in.timestamp);
    return(false);
}


bool MapFsmore::AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T){
    Eigen::Vector3f p1_1,p1_2;
    float k;
    Line l_o=ForceToLine(F,M,p1_1,p1_2,k);
    Line l_m=l_o.Transform(T);

    std::vector<PType> ray;
    oct_obj->computeRay(PType(l_m.p1.x(), l_m.p1.y(), l_m.p1.z()),
                        PType(l_m.p2.x(), l_m.p2.y(), l_m.p2.z()),
                        ray);


    //    PCType::VectorType ray;
    //    Eigen::Vector3f dist=(l_m.p2-l_m.p1);
    //    int n_points=(int) dist.norm()/line_res;
    //    for (int i=0;i<n_points;i++){
    //        PType p;
    //        Eigen::Vector3f p2=l_m.p1+((float) i*line_res*l_m.dir.normalized());
    //        p.x=p2.x();
    //        p.y=p2.y();
    //        p.z=p2.z();
    //        ray.push_back(p);
    //    }
    //    //oct_map->getIntersectedVoxelCenters(l_m.p1,l_m.p2,ray);

    //KeyType kk=oct_map->coordToKey(PType(0.2f,0.0f,0.2f));
    //printf("KK: %d\n", KeyHasher(kk));


    bool lmap_exists=LineExists(lines_map,l_m);
    bool lobj_exists=LineExists(lines_obj,l_o);

    if(lmap_exists && lobj_exists){
        return(false);
    }
    else{
        lines_map.push_back(l_m);
        lines_obj.push_back(l_o);
    }

    float tot_prob_line=0.0f;

    for (PCType::iterator it=ray.begin();it!=ray.end();it++){
        size_t key_map,key_obj;
        Voxel *v_m,*v_o;

        //Do Map things
        key_map=KeyHasher(oct_map->coordToKey(*it));
        if(map_map.find(key_map)==map_map.end()){
            PType p= *it;
            v_m=new Voxel(oct_map,p,key_map);
            map_map.insert(std::pair<size_t,Voxel>(key_map,*v_m));
            v_m=&map_map.at(key_map);
            v_m->setLikelihood(1.0f/(float) ray.size());

            //pc_map->push_back(p);
            //oct_map->addPointToCloud(p,pc_map);
        }
        else{
            //oct_map->findLeafAtPoint(*it)->
            //     key_map=oct_map->getGridCoordinates(it->x,it->y,it->z);
            //if(map_map.find(KeyHasher(key))!=map_map.end()){
            v_m=&(map_map.at(key_map));
            //}
        }

        //std::shared_ptr<Line> p_lm(&lines_map.back());
        v_m->intersecting.push_back(&lines_map.back());
        lines_map.back().voxels.push_back(v_m);
        //lines_map.back().voxels.push_back(v_m);



        //Do object things
        PType p_o = v_m->TransformCoord(T.inverse());
        key_obj=KeyHasher(oct_map->coordToKey(p_o));

        if(map_obj.find(key_obj)==map_obj.end()){
            v_o=new Voxel(oct_obj,p_o,key_obj);
            map_obj.insert(std::pair<size_t,Voxel>(key_obj,*v_o));
            v_o=&(map_obj.at(key_obj));
            v_o->setLikelihood(1.0f/((float) ray.size()));
        }
        else {
            v_o=&(map_obj.at(key_obj));
        }
        v_o->intersecting.push_back(&lines_obj.back());
        lines_obj.back().voxels.push_back(v_o);


        float prob_contact=v_m->getLikelihood()*v_o->getLikelihood();
        lines_map.back().prob_contact.push_back(prob_contact);
        lines_obj.back().prob_contact.push_back(prob_contact);
        tot_prob_line+=prob_contact;

        //float lkl1=v_m->getLikelihood();
        //float lkl2=v_o->getLikelihood();
        //v_m->setLikelihood(lkl1+0.1);
    }
    //Normalize ProbContact:
    if((lines_map.back().prob_contact.size() != lines_map.back().prob_contact.size() )
            || (lines_map.back().prob_contact.size()!=lines_map.back().voxels.size() )
            || (lines_obj.back().prob_contact.size()!=lines_obj.back().voxels.size() )){
        printf("\n\n\n\n SIZES DON'T MATCH!!!!\n\n\n\n l: %zu %zu v: %zu %zu\n\n\n",
               lines_map.back().prob_contact.size(),
               lines_map.back().prob_contact.size(),
               lines_map.back().voxels.size(),
               lines_obj.back().voxels.size()
               );
        return(false);
    }

    for (size_t i=0;i<lines_map.back().prob_contact.size();i++) {
        lines_map.back().prob_contact.at(i)/=tot_prob_line;
        float max_lik=std::max(lines_map.back().voxels.at(i)->getLikelihood(),lines_map.back().prob_contact.at(i));
        lines_map.back().voxels.at(i)->setLikelihood(max_lik);
        lines_obj.back().voxels.at(i)->setLikelihood(max_lik);
    }

    UpdateProbs(&lines_map.back(),oct_map,oct_obj,map_map,map_obj,T.inverse(),1);
    UpdateProbs(&lines_obj.back(),oct_obj,oct_map,map_obj,map_map,T,1);



    return(true);
}


bool MapFsmore::AddEmpty(Eigen::Affine3f T){
    if(map_map.size()>map_obj.size()){
        UpdateFree(oct_obj,oct_map,map_obj,map_map,T);
    }
    else{
        UpdateFree(oct_map,oct_obj,map_map,map_obj,T.inverse());
    }
    return(true);
}

void MapFsmore::UpdateFree(OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T){
    for (std::map<size_t,Voxel>::iterator it = map.begin();it!=map.end();it++){

        PType p=it->second.TransformCoord(T);

        std::map<size_t,Voxel>::iterator it2=other_map.find(KeyHasher(other_oct->coordToKey(p)));
        if(it2!=other_map.end()){
           float P1=it->second.getLikelihood();
           float P2=it2->second.getLikelihood();
           float prod=1-(P1*P2);
           if(prod == 0.0f) prod=0.0000001f;
           it->second.setLikelihood((P1*(1-P2))/prod);
           it2->second.setLikelihood((P2*(1-P1))/prod);
        }
    }

}

void MapFsmore::UpdateProbs(Line *line,OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T, int depth){

    float prod=1.0;
    float sum=0;
    for (size_t i=0;i<line->voxels.size();i++){
        if(depth!=0){ //normalize intersecting lines
            for(size_t j=0;j<line->voxels.at(i)->intersecting.size();j++){
                if(line->voxels.at(i)->intersecting.at(j)->p1!=line->p1){ //but only other lines (not this)
                    UpdateProbs(line->voxels.at(i)->intersecting.at(j),oct,other_oct,map,other_map,T,0); // send zero depth to avoid infinite recursion.
                }
            }
        }
        Eigen::Vector3f other_point=T*line->voxels.at(i)->p;
        size_t other_key=KeyHasher(other_oct->coordToKey(other_point.x(),other_point.y(),other_point.z()));
        float other_prob=0;
        if(other_map.find(other_key)!=other_map.end()){
            other_prob=other_map.at(other_key).likelihood;
        }
        else{
            ///TODO: Add voxel there instead
            other_prob=1.0f/line->voxels.size();
        }
        //float prob=line->voxels.at(i)->likelihood*other_prob;
        float prob=line->voxels.at(i)->likelihood;
        prod*=(1-prob);
        sum+=prob;
    }
    for (size_t i=0;i<line->voxels.size();i++){
        float likl1=line->voxels.at(i)->getLikelihood();
        line->voxels.at(i)->setLikelihood(likl1/(1-prod));
    }
}


void MapFsmore::NormalizeLine(Line l){
    float tot=0.0f;
    for (size_t i = 0; i < l.voxels.size(); ++i) {
        tot+=l.voxels.at(i)->getLikelihood();
    }
    for (size_t i = 0; i < l.voxels.size(); ++i) {
        l.voxels.at(i)->setLikelihood(l.voxels.at(i)->getLikelihood()/tot);
    }
}

void MapFsmore::CleanupLines(){
    time_t now;
    time(&now);
    for(size_t i=0;i<lines_map.size();i++){
        if(difftime(now,lines_map.at(i).timestamp)>decay_time){
            DeleteLine(oct_map,map_map,lines_map,i,true);
        }
    }
    for(size_t i=0;i<lines_obj.size();i++){
        if(difftime(now,lines_obj.at(i).timestamp)>decay_time){
            DeleteLine(oct_obj,map_obj,lines_obj,i,true);
        }
    }
}

void MapFsmore::DeleteLine(OctTypePtr oct,std::map<size_t,Voxel> &map, std::vector<Line> &lines,size_t line_nr,bool keep_maxlik){
    //delete voxels* from deleted line
    //delete voxel from voxel_map
    //delete line* from each voxel
    //delete line from lines_map

    ///TODO keep zero likelihood points

    Voxel* best_voxel = nullptr;
    float max_lik=0;
    for(size_t i=0;i<lines.at(line_nr).voxels.size();i++){
        KeyType key=lines.at(line_nr).voxels.at(i)->key;
        if(map.find(KeyHasher(key))!=map.end()){
            Voxel v(map.find(KeyHasher(key))->second);
            if(keep_maxlik && v.likelihood>=max_lik){
                //best_voxel=new Voxel(map.find(KeyHasher(key))->second);
                best_voxel=new Voxel(v);

                //max_lik=map.find(KeyHasher(key))->second.likelihood;
                max_lik=v.likelihood;
            }
            if(v.likelihood<0.99f){
                //    map.erase(KeyHasher(key));
                //    oct->deleteNode(key);
            }
        }
    }
    if(keep_maxlik){
        for (size_t i=0;i<best_voxel->intersecting.size();i++){
            //if(&(lines.at(i))==best_voxel->intersecting.at(i)){
            if(&(lines.at(line_nr))==best_voxel->intersecting.at(i)){
                best_voxel->intersecting.erase(best_voxel->intersecting.begin()+i); //erase the reference in this voxel to this line
            }
        }
        oct->setNodeValue(best_voxel->key,best_voxel->likelihood);
        map.insert(std::pair<size_t,Voxel>(KeyHasher(best_voxel->key),*best_voxel));
    }
    lines.erase(lines.begin()+line_nr);
    return;
}



void MapFsmore::ComputeProbabilities(){
    for(size_t i=0;i<lines_map.size();i++){
        NormalizeLine(lines_map.at(i));
    }
    // for(size_t i=0;i<lines_obj.size();i++){
    //     NormalizeLine(lines_obj.at(i));
    // }
}

pcl::PointCloud<pcl::PointXYZI> MapFsmore::getPointCloud(OctTypePtr octree){
    pcl::PointCloud<pcl::PointXYZI> pc;

    for(OctType::iterator it = octree->begin();it!=octree->end();it++){
        if(it->getValue()>0.5){
            pcl::PointXYZI p;
            p.x=it.getX();
            p.y=it.getY();
            p.z=it.getZ();
            p.intensity=it->getValue();// Occupancy();//getLogOdds();
            pc.push_back(p);
        }
        //pcl::octree::OctreeLeafNode<PType> n;

    }
    return(pc);
}


pcl::PointCloud<pcl::PointXYZI> MapFsmore::getMapPointCloud(){
    pcl::PointCloud<pcl::PointXYZI>  pc;
    pc=getPointCloud(oct_map);
    return(pc);
}

pcl::PointCloud<pcl::PointXYZI> MapFsmore::getObjectPointCloud(){
    pcl::PointCloud<pcl::PointXYZI>  pc;
    pc=getPointCloud(oct_obj);
    return(pc);
}
