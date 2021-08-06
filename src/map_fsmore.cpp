#include <fsmore/map_fsmore.h>
#include <sstream>
#include <omp.h>
#include <sys/time.h>   // for gettimeofday()
#include <chrono>

MapFsmore::MapFsmore()
    : pc_obj(new PCType),
      pc_map(new PCType){

    oct_map = OctTypePtr(new OctType(line_res));
    oct_obj = OctTypePtr(new OctType(line_res));
    omp_set_num_threads(4);
}

void MapFsmore::resetMap(double resolution){
    line_res=resolution;
    oct_map = OctTypePtr(new OctType(line_res));
    oct_obj = OctTypePtr(new OctType(line_res));
    map_map.clear();
    map_obj.clear();
    lines_map.clear();
    lines_obj.clear();
}

std::size_t MapFsmore::KeyHasher(KeyType key_arg){
    std::size_t seed = 0;
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

bool MapFsmore::CompareLines(Line l1,Line l2){
    float dp=l1.InnerProd(l2);
    if(dp>same_line_tol) {
        Eigen::Vector3f dir2=l2.p2-l1.p1;
        Eigen::Vector3f dir2b=l2.p1-l1.p1;
        if(dir2b.norm()>dir2.norm()) dir2=dir2b; // find the farthest point to draw vector
        if(fabs(l1.dir.dot(dir2.normalized()))>same_line_tol){
            return(true);
        }
    }
    return(false);
}

bool MapFsmore::LineExists(std::list<Line> &lines,Line &l_in){
    time_t now;
    time(&now);
    for (auto it=lines.begin();it!=lines.end();it++){
        if(difftime(now,it->timestamp)<5.0 && CompareLines(l_in,*it)){
            time(&(it->timestamp));
            return true;
        }
    }
    time(&l_in.timestamp);
    return(false);
}


bool MapFsmore::AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T,Line &l_map, Line &l_obj){
    Eigen::Vector3f p1_1,p1_2;
    float k;
    Line l_o=ForceToLine(F,M,p1_1,p1_2,k);
    Line l_m=l_o.Transform(T);
    l_map=l_m;
    l_obj=l_o;

    std::vector<PType> ray;
    oct_obj->computeRay(PType(l_m.p1.x(), l_m.p1.y(), l_m.p1.z()),
                        PType(l_m.p2.x(), l_m.p2.y(), l_m.p2.z()),
                        ray);

    auto begin2=std::chrono::high_resolution_clock::now();
    bool lmap_exists=LineExists(lines_map,l_m);
    bool lobj_exists=LineExists(lines_obj,l_o);
    printf("LE %ld \n",std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin2).count());


    if(lmap_exists && lobj_exists){
        return(false);
    }
    else{
        lines_map.push_back(l_m);
        lines_obj.push_back(l_o);
    }

    for (PCType::iterator it=ray.begin();it!=ray.end();it++){
        size_t key_map=KeyHasher(oct_map->coordToKey(*it));
        Voxel *v;
        if(map_map.find(key_map)==map_map.end()){
            v=new Voxel(oct_map,*it,key_map);
            v->setLikelihood(1.0f/ static_cast<float>(ray.size()));
            map_map.insert(std::pair<size_t,Voxel>(key_map,*v));
        }
        v=&(map_map.at(key_map));
        v->intersecting.push_back(&lines_map.back());
        lines_map.back().voxels.push_back(v);
    }
    UpdateProbs(&lines_map.back(),oct_map,oct_obj,map_map,map_obj,T.inverse(),1);
    return(true);
}


bool MapFsmore::AddEmpty(Eigen::Affine3f T){
    UpdateFree(oct_obj,oct_map,map_obj,map_map,T);
    return(true);
}

void MapFsmore::UpdateFree(OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T){
    printf("Map sizes (free): %zu %zu \n",map.size(),other_map.size());

    //auto begin= std::chrono::high_resolution_clock::now();
    int map_size=static_cast<int>(map.size());
    int n_threads=static_cast<int>(omp_get_max_threads());
    omp_lock_t writelock;
    omp_init_lock(&writelock);


#pragma omp parallel for
    for (int thr=0;thr<n_threads;thr++){
        //auto begin2= std::chrono::high_resolution_clock::now();
        int tid = omp_get_thread_num();
        auto it = map.begin();
        std::advance(it, tid*map_size/n_threads);
        for (int i = 0; i < static_cast<int>(map.size())/n_threads; i++){
            //for (std::map<size_t,Voxel>::iterator it = map.begin();it!=map.end();it++){
            PType p=it->second.TransformCoord(T);
            //        std::cout << "1 " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin).count() << "  " << tid << "\n";
            std::map<size_t,Voxel>::iterator it2=other_map.find(KeyHasher(other_oct->coordToKey(p)));
            //        std::cout << "2 " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin).count() << "  " << tid << "\n";
            if(it2!=other_map.end()){
                float P1=it->second.getLikelihood();
                float P2=it2->second.getLikelihood();
                float prod=1-(P1*P2);
                if(prod == 0.0f) prod=0.0000001f;
                omp_set_lock(&writelock);
                //#pragma omp critical
                {
                    it->second.setLikelihood((P1*(1-P2))/prod);
                    it2->second.setLikelihood((P2*(1-P1))/prod);
                }
                omp_unset_lock(&writelock);
                //            std::cout << "3 " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin).count() << " " << tid << "\n";
            }
            else{
                //            std::cout << "4 " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin).count() << " " << tid << "\n";
            }
            it++;
        }
        omp_destroy_lock(&writelock);
        //printf("000 %ld %d / %d \n",std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin2).count(),tid,n_threads);
        //    std::cout << "5 " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin).count() << " " << "\n+++++++++++\n";
    }
}

void MapFsmore::UpdateProbs(Line *line,OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T, int depth){

    float prod2=1.0;
    bool paired=false;
    omp_lock_t writelock;
    omp_init_lock(&writelock);

#pragma omp parallel for
    for (size_t i=0;i<line->voxels.size();i++){
        //        if(depth!=0){ //normalize intersecting lines
        //            for(auto it = line->voxels.at(i)->intersecting.begin();it!=line->voxels.at(i)->intersecting.end();it++){
        //                if((*it)->p1!=line->p1){
        //                    //UpdateProbs(*it,oct,other_oct,map,other_map,T,0); // send zero depth to avoid infinite recursion.
        //                }
        //            }
        //        }
        Eigen::Vector3f other_point=T*line->voxels.at(i)->p;
        size_t other_key=KeyHasher(other_oct->coordToKey(static_cast<double>(other_point.x()),static_cast<double>(other_point.y()),static_cast<double>(other_point.z())));

        float other_prob=0;

        if(other_map.find(other_key)!=other_map.end()){
            //other_prob=std::max(other_map.at(other_key).getLikelihood(),1.0f/line->voxels.size());
            other_prob=other_map.at(other_key).getLikelihood();
            paired=true;
        }
        else{
            other_prob=1.0f/line->voxels.size();
            Voxel v_o(other_oct,PType(other_point.x(),other_point.y(),other_point.z()),other_key);
            omp_set_lock(&writelock);
            //#pragma omp critical
            {
                v_o.setLikelihood(other_prob);
                other_map.insert(std::pair<size_t,Voxel>(other_key,v_o));
            }
            omp_unset_lock(&writelock);
        }

        //float prob=std::max(line->voxels.at(i)->getLikelihood(),1.0f/line->voxels.size());
        float prob=line->voxels.at(i)->getLikelihood();
        //float prob2=line->voxels.at(i)->getLikelihood()*other_prob;
        float prob2=(1-(prob*other_prob));

        omp_set_lock(&writelock);
        //#pragma omp critical
        {
            prod2*=prob2;
        }
        omp_unset_lock(&writelock);

    }

#pragma omp parallel for
    for (size_t i=0;i<line->voxels.size();i++){
        Eigen::Vector3f other_point=T*line->voxels.at(i)->p;
        size_t other_key=KeyHasher(other_oct->coordToKey(static_cast<double>(other_point.x()),static_cast<double>(other_point.y()),static_cast<double>(other_point.z())));

        float likl1=line->voxels.at(i)->getLikelihood();
        float likl2=other_map.at(other_key).getLikelihood();

        float new_likl1=((likl1*likl2)+((1-((prod2/(1-likl1*likl2))))*(likl1*(1-likl2))))/(1-prod2);
        float new_likl2=((likl2*likl1)+((1-((prod2/(1-likl2*likl1))))*(likl2*(1-likl1))))/(1-prod2);
        omp_set_lock(&writelock);
        //#pragma omp critical
        {
            line->voxels.at(i)->setLikelihood(new_likl1);
            other_map.at(other_key).setLikelihood(new_likl2);
        }
        omp_unset_lock(&writelock);
    }
}

void MapFsmore::CleanupLines(){
    time_t now;
    time(&now);
    for (auto it=lines_map.begin();it!=lines_map.end();it++){
        if(difftime(now,(*it).timestamp)>decay_time){
            DeleteLine(oct_map,map_map,lines_map,(*it));
            lines_map.erase(it--);
        }
    }
    for (auto it=lines_obj.begin();it!=lines_obj.end();it++){
        if(difftime(now,(*it).timestamp)>decay_time){
            DeleteLine(oct_obj,map_obj,lines_obj,(*it));
            lines_obj.erase(it--);
        }
    }
}

void MapFsmore::DeleteLine(OctTypePtr oct,std::map<size_t,Voxel> &map, std::list<Line> &lines,Line& line_nr){
    //Delete the pointer to the intersecting line in each voxel
    //Delete the line from the lines map
    for (auto it=line_nr.voxels.begin();it!=line_nr.voxels.end();it++){
        for (auto it2=(*it)->intersecting.begin();it2!=(*it)->intersecting.end();it2++){
            if(CompareLines(*(*it2),line_nr)){
                (*it)->intersecting.erase(it2--);
            }
        }
    }
}

/*
void MapFsmore::ComputeProbabilities(){
    for (auto it=lines_map.begin();it!=lines_map.end();it++){
        NormalizeLine(*it);
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
*/
pcl::PointCloud<pcl::PointXYZI> MapFsmore::getPointCloud(OctTypePtr octree,double min_intensity){
    pcl::PointCloud<pcl::PointXYZI> pc;

    int octree_size=static_cast<int>(octree->getNumLeafNodes());
    int n_threads=static_cast<int>(omp_get_max_threads());
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    //    for(OctType::iterator it = octree->begin();it!=octree->end();it++){
#pragma omp parallel for
    for (int thr=0;thr<n_threads;thr++){
        int tid = omp_get_thread_num();
        OctType::iterator it = octree->begin();
        std::advance(it, tid*octree_size/n_threads);
        for (int i = 0; i < static_cast<int>(octree_size)/n_threads; i++){
            if(it->getOccupancy()>=min_intensity){
                pcl::PointXYZI p;
                p.x=static_cast<float>(it.getX());
                p.y=static_cast<float>(it.getY());
                p.z=static_cast<float>(it.getZ());
                p.intensity=static_cast<float>(it->getOccupancy());//Value();// Occupancy();//getLogOdds();
//#pragma omp critical
                {
                omp_set_lock(&writelock);
                pc.push_back(p);
                omp_unset_lock(&writelock);
                }
            }

            it++;
        }
    }
    omp_destroy_lock(&writelock);
    return(pc);
}


pcl::PointCloud<pcl::PointXYZI> MapFsmore::getMapPointCloud(double min_intensity){
    pcl::PointCloud<pcl::PointXYZI>  pc;
    pc=getPointCloud(oct_map,min_intensity);
    return(pc);
}

pcl::PointCloud<pcl::PointXYZI> MapFsmore::getObjectPointCloud(double min_intensity){
    pcl::PointCloud<pcl::PointXYZI>  pc;
    pc=getPointCloud(oct_obj,min_intensity);
    return(pc);
}
