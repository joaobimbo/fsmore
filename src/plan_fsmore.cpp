#include <fsmore/plan_fsmore.h>

PlanFsmore::PlanFsmore()
{

}


void PlanFsmore::setMapOctree(octomap::OcTree in){
    oct_map =std::make_shared<octomap::OcTree>(in);
    col_map = new fcl::OcTreef(oct_map);
}

void PlanFsmore::setObjOctree(octomap::OcTree in){
    //oct_obj=new octomap::OcTree(in);
    oct_obj =std::make_shared<octomap::OcTree>(in);
    col_obj = new fcl::OcTreef(oct_obj);
}
