#include <fsmore/plan_fsmore.h>

PlanFsmore::PlanFsmore()
{

}


void PlanFsmore::setMapOctree(octomap::OcTree in){
    oct_map=new octomap::OcTree(in);
}

void PlanFsmore::setObjOctree(octomap::OcTree in){
    oct_obj=new octomap::OcTree(in);
}
