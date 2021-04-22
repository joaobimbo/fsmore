#ifndef PLAN_FSMORE_H
#define PLAN_FSMORE_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <fsmore/map_fsmore.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>

class PlanFsmore
{
public:
    PlanFsmore();
    virtual void setupPlanner() = 0;
    virtual std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal) = 0;
    virtual void setBounds(Eigen::Vector3f min,Eigen::Vector3f max) = 0;
    virtual bool isStateValid(const ompl::base::State *state) = 0;
    virtual void setStartAndGoal(geometry_msgs::Pose start,geometry_msgs::Pose goal) = 0;

    OctType *oct_map,*oct_obj;

    void setMapOctree(octomap::OcTree in);
    void setObjOctree(octomap::OcTree in);


protected:
    std::shared_ptr<ompl::base::SpaceInformation> si;

};

#endif // PLAN_FSMORE_H
