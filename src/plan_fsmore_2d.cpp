#include <fsmore/plan_fsmore_2d.h>
//#include <fcl/octree.h>
//#include <fcl/collision.h>

PlanFsmore_2D::PlanFsmore_2D()
{

}

void PlanFsmore_2D::setupPlanner(){


}


std::vector<geometry_msgs::Pose> PlanFsmore_2D::getPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal){
    std::vector<geometry_msgs::Pose> plan;
    plan.push_back(start);
    plan.push_back(goal);



    printf("Planner2D\n");


    printf("Sizes: %d %d\n",oct_map->getNumLeafNodes(),oct_obj->getNumLeafNodes());

    return(plan);



}
