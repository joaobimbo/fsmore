#ifndef PLAN_FSMORE_H
#define PLAN_FSMORE_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <fsmore/map_fsmore.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerStatus.h>

#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

class PlanFsmore
{
public:
    PlanFsmore();
    virtual void setupPlanner() = 0;
    //virtual std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal) = 0;
    virtual ompl::base::PlannerStatus getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal,geometry_msgs::PoseArray &solution,geometry_msgs::PoseArray &vertexes) = 0;
    virtual void setBounds(Eigen::Vector3d min,Eigen::Vector3d max) = 0;
    virtual bool isStateValid(const ompl::base::State *state) = 0;
    virtual void setStartAndGoal(geometry_msgs::Pose start,geometry_msgs::Pose goal) = 0;
    virtual void setPlannerOptions(double step_siz, double timeout, Eigen::Vector3d min_bound, Eigen::Vector3d max_bound) = 0;
    virtual bool isValid(geometry_msgs::Pose pose) = 0;


    OctTypePtr oct_map,oct_obj;
    fcl::OcTreef *col_map,*col_obj;

    void setMapOctree(octomap::OcTree in);
    void setObjOctree(octomap::OcTree in);

protected:
    double step_size,plan_timeout;

    std::shared_ptr<ompl::base::SpaceInformation> si;
    std::shared_ptr<ompl::base::ProblemDefinition> pdef;
    std::shared_ptr<ompl::base::Planner> plan_ptr;

};

#endif // PLAN_FSMORE_H
