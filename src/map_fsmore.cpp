#include <fsmore/map_fsmore.h>
#include <sstream>
#include <omp.h>
#include <sys/time.h>   // for gettimeofday()
#include <chrono>

MapFsmore::MapFsmore()
    : pc_obj(new PCType),
      pc_map(new PCType){
    //oct_map = OctType::Ptr(new OctType(0.01));
    //oct_obj = OctType::Ptr(new OctType(0.01));
    //oct_map = OctType::Ptr(new OctType());
    //oct_obj = OctType::Ptr(new OctType());
    oct_map = OctTypePtr(new OctType(line_res));
    oct_obj = OctTypePtr(new OctType(line_res));
    //oct_map->setClampingThresMin(0.00);
    //oct_obj->setClampingThresMin(0.00);


    //lines_map.reserve(10000);
    //lines_obj.reserve(10000);

    //oct_map->setLeafSize(line_res,line_res,line_res);
    //oct_map->setInputCloud(pc_map);
    //oct_map->setSaveLeafLayout(true);
    //oct_map->addPointsFromInputCloud();

    //oct_obj->setLeafSize(line_res,line_res,line_res);
    //oct_obj->setInputCloud(pc_obj);
    //oct_obj->setSaveLeafLayout(true);
    //oct_obj->addPointsFromInputCloud();
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
    for (auto it=lines.begin();it!=lines.end();it++){
        if(CompareLines(l_in,*it)){
            time(&(it->timestamp));
            return true;
        }
    }
    //    for (size_t i=0;i<lines.size();i++){
    //        if(CompareLines(l_in,lines.at(i))){
    //            time(&lines.at(i).timestamp);
    //            return(true);
    //        }
    //    }
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

    /*

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
    */
    //Normalize ProbContact:
    /*    if((lines_map.back().prob_contact.size() != lines_map.back().prob_contact.size() )
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
    }

    for (size_t i=0;i<lines_obj.back().prob_contact.size();i++) {
        lines_obj.back().prob_contact.at(i)/=tot_prob_line;
        float max_lik=std::max(lines_obj.back().voxels.at(i)->getLikelihood(),lines_obj.back().prob_contact.at(i));
        lines_obj.back().voxels.at(i)->setLikelihood(max_lik);
    }
*/
    // UpdateProbs(&lines_map.back(),oct_map,oct_obj,map_map,map_obj,T.inverse(),1);
    //  UpdateProbs(&lines_obj.back(),oct_obj,oct_map,map_obj,map_map,T,1);

    // return(true);
}


bool MapFsmore::AddEmpty(Eigen::Affine3f T){
    //if(map_map.size()>map_obj.size()){
    UpdateFree(oct_obj,oct_map,map_obj,map_map,T);
    //}
    //else{
    //UpdateFree(oct_map,oct_obj,map_map,map_obj,T.inverse());
    //}
    return(true);
}

void MapFsmore::UpdateFree(OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T){
    printf("Map sizes (free): %zu %zu \n",map.size(),other_map.size());

    auto begin= std::chrono::high_resolution_clock::now();
    int map_size=static_cast<int>(map.size());
    int n_threads=static_cast<int>(omp_get_max_threads());
    omp_lock_t writelock;
    omp_init_lock(&writelock);


#pragma omp parallel for
    for (int thr=0;thr<n_threads;thr++){
        auto begin2= std::chrono::high_resolution_clock::now();
        int tid = omp_get_thread_num();
        auto it = map.begin();
        std::advance(it, tid*map_size/n_threads);
        for (int i = 0; i < static_cast<int>(map.size())/n_threads; i++){
            //for (std::map<size_t,Voxel>::iterator it = map.begin();it!=map.end();it++){
            //printf("This is thread %i announcing that the number of launched threads is %i\n", tid, omp_get_num_threads());
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
        printf("000 %ld %d / %d \n",std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin2).count(),tid,n_threads);
        //    std::cout << "5 " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-begin).count() << " " << "\n+++++++++++\n";
    }
}

void MapFsmore::UpdateProbs(Line *line,OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T, int depth){

    //float prod=1.0;
    //float sum=0;
    float prod2=1.0;
    //float sum2=0;
    bool paired=false;
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    //std::ostringstream debug_1,debug_2;
#pragma omp parallel for
    for (size_t i=0;i<line->voxels.size();i++){
        //        if(depth!=0){ //normalize intersecting lines
        //            //for(size_t j=0;j<line->voxels.at(i)->intersecting.size();j++){
        //            //    if(line->voxels.at(i)->intersecting.at(j)->p1!=line->p1){ //but only other lines (not this)
        //            //        UpdateProbs(line->voxels.at(i)->intersecting.at(j),oct,other_oct,map,other_map,T,0); // send zero depth to avoid infinite recursion.
        //            //    }
        //            //}
        //            for(auto it = line->voxels.at(i)->intersecting.begin();it!=line->voxels.at(i)->intersecting.end();it++){
        //                if((*it)->p1!=line->p1){
        //                    //UpdateProbs(*it,oct,other_oct,map,other_map,T,0); // send zero depth to avoid infinite recursion.
        //                }
        //            }
        //        }
        Eigen::Vector3f other_point=T*line->voxels.at(i)->p;
        size_t other_key=KeyHasher(other_oct->coordToKey(other_point.x(),other_point.y(),other_point.z()));

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

        //prod*=(1-prob);
        omp_set_lock(&writelock);
        //#pragma omp critical
        {
            prod2*=prob2;
        }
        omp_unset_lock(&writelock);

        //sum+=prob;
        //sum2+=prob2;
        //debug_1 << prob << ",";
        //debug_2 << other_prob << ",";
        //printf("%f, ",prob);
    }
    //std::cout << debug_1.str() << "\n" << debug_2.str() << "\n";
    //printf("prd=%f\n",prod2);
    //debug_1.str("");
    //debug_2.str("");
#pragma omp parallel for
    for (size_t i=0;i<line->voxels.size();i++){
        Eigen::Vector3f other_point=T*line->voxels.at(i)->p;
        size_t other_key=KeyHasher(other_oct->coordToKey(other_point.x(),other_point.y(),other_point.z()));

        float likl1=line->voxels.at(i)->getLikelihood();
        float likl2=other_map.at(other_key).getLikelihood();

        //float new_likl=(likl1)/(1-prod);
        //float Pc=(likl1*likl2)/(1-prod2);
        //line->voxels.at(i)->setLikelihood(Pc+(likl1*(1-likl2))*(1-Pc));
        //other_map.at(other_key).setLikelihood(Pc+(likl2*(1-likl1))*(1-Pc));
        //line->voxels.at(i)->setLikelihood(Pc);
        //other_map.at(other_key).setLikelihood(Pc);


        float new_likl1=((likl1*likl2)+((1-((prod2/(1-likl1*likl2))))*(likl1*(1-likl2))))/(1-prod2);
        float new_likl2=((likl2*likl1)+((1-((prod2/(1-likl2*likl1))))*(likl2*(1-likl1))))/(1-prod2);
        //printf("1: %f -> %f 2: %f -> %f | %f\n",likl1,new_likl1,likl2,new_likl2,prod);
        //printf("%f, ",new_likl1);
        omp_set_lock(&writelock);
        //#pragma omp critical
        {
            line->voxels.at(i)->setLikelihood(new_likl1);
            //printf("l: %f %f\n",new_likl2 , line->voxels.at(i)->getLikelihood());
            other_map.at(other_key).setLikelihood(new_likl2);
        }
        omp_unset_lock(&writelock);

        //debug_1 << new_likl1 << ",";
        //debug_2 << new_likl2 << ",";
    }
    //std::cout << debug_1.str() << "\n" << debug_2.str() << "\n==\n";

    //printf("\n");
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


    //    for(size_t i=0;i<lines_map.size();i++){
    //        if(difftime(now,lines_map.at(i).timestamp)>decay_time){
    //            DeleteLine(oct_map,map_map,lines_map,i,true);
    //        }
    //    }
    //    for(size_t i=0;i<lines_obj.size();i++){
    //        if(difftime(now,lines_obj.at(i).timestamp)>decay_time){
    //            DeleteLine(oct_obj,map_obj,lines_obj,i,true);
    //        }
    //    }
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
void MapFsmore::DeleteLine3(OctTypePtr oct,std::map<size_t,Voxel> &map, std::vector<Line> &lines,size_t line_nr,bool keep_maxlik){
    //Delete the pointer to the intersecting line in each voxel
    //Delete the line from the lines map
    for(size_t i=0;i<lines.at(line_nr).voxels.size();i++){
        for (size_t j=0; j<lines.at(line_nr).voxels.at(i)->intersecting.size();j++){
            if(CompareLines(*(lines.at(line_nr).voxels.at(i)->intersecting.at(j)),lines.at(line_nr))){
                lines.at(line_nr).voxels.at(i)->intersecting.erase(lines.at(line_nr).voxels.at(i)->intersecting.begin()+j);
            }
        }
    }
    lines.erase(lines.begin()+line_nr);
}

void MapFsmore::DeleteLine2(OctTypePtr oct,std::map<size_t,Voxel> &map, std::vector<Line> &lines,size_t line_nr,bool keep_maxlik){
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
        oct->setNodeValue(best_voxel->key,log(best_voxel->likelihood/(1-best_voxel->likelihood)));


        map.insert(std::pair<size_t,Voxel>(KeyHasher(best_voxel->key),*best_voxel));
    }
    lines.erase(lines.begin()+line_nr);
    return;
}
*/


void MapFsmore::ComputeProbabilities(){
    for (auto it=lines_map.begin();it!=lines_map.end();it++){
        NormalizeLine(*it);
    }
    //    for(size_t i=0;i<lines_map.size();i++){
    //        NormalizeLine(lines_map.at(i));
    //    }
    // for(size_t i=0;i<lines_obj.size();i++){
    //     NormalizeLine(lines_obj.at(i));
    // }
}

pcl::PointCloud<pcl::PointXYZI> MapFsmore::getPointCloud(OctTypePtr octree,double min_intensity){
    pcl::PointCloud<pcl::PointXYZI> pc;

    for(OctType::iterator it = octree->begin();it!=octree->end();it++){
        if(it->getOccupancy()>=min_intensity){
            pcl::PointXYZI p;
            p.x=static_cast<float>(it.getX());
            p.y=static_cast<float>(it.getY());
            p.z=static_cast<float>(it.getZ());
            p.intensity=static_cast<float>(it->getOccupancy());//Value();// Occupancy();//getLogOdds();
            pc.push_back(p);
        }
        //pcl::octree::OctreeLeafNode<PType> n;

    }
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
