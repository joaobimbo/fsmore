#ifndef PLAN_FSMORE_2D_H
#define PLAN_FSMORE_2D_H
#include <fsmore/plan_fsmore.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>

class PlanFsmore_2D : public PlanFsmore
{
public:
    PlanFsmore_2D();
    void setupPlanner();
    std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal);
    void setBounds(Eigen::Vector3f min, Eigen::Vector3f max);
    bool isStateValid(const ompl::base::State *state);
    void setStartAndGoal(geometry_msgs::Pose start, geometry_msgs::Pose goal_pose);

protected:
    std::shared_ptr<ompl::base::SE2StateSpace> space;
    std::shared_ptr<ompl::base::ProblemDefinition> pdef;
    ompl::base::RealVectorBounds bounds;
    geometry_msgs::Pose pose2Dto3D(double x,double y,double z,double ang);
    float height_z=0.2;

};

#endif // PLAN_FSMORE_2D_H
