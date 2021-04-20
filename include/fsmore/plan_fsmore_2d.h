#ifndef PLAN_FSMORE_2D_H
#define PLAN_FSMORE_2D_H
#include <fsmore/plan_fsmore.h>

class PlanFsmore_2D : public PlanFsmore
{
public:
    PlanFsmore_2D();
    void setupPlanner();
    std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal);
    void setMapOctree(octomap::OcTree in);
    void setObjOctree(octomap::OcTree in);
protected:



};

#endif // PLAN_FSMORE_2D_H
