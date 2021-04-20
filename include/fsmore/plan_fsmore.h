#ifndef PLAN_FSMORE_H
#define PLAN_FSMORE_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <fsmore/map_fsmore.h>

class PlanFsmore
{
public:
    PlanFsmore();
    virtual void setupPlanner() = 0;
    virtual std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal) = 0;
    OctType *oct_map,*oct_obj;

    void setMapOctree(octomap::OcTree in);
    void setObjOctree(octomap::OcTree in);

};

#endif // PLAN_FSMORE_H
