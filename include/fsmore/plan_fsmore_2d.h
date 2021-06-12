#ifndef PLAN_FSMORE_2D_H
#define PLAN_FSMORE_2D_H
#include <fsmore/plan_fsmore.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>


class PlanFsmore_2D : public PlanFsmore
{
public:
    PlanFsmore_2D();
    void setupPlanner();
    //std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal, ompl::base::PlannerData &data);
    //std::vector<geometry_msgs::Pose> getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal);
    ompl::base::PlannerStatus getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal, geometry_msgs::PoseArray &solution, geometry_msgs::PoseArray &vertexes);
    void setBounds(Eigen::Vector3d min, Eigen::Vector3d max);
    bool isStateValid(const ompl::base::State *state);
    void setStartAndGoal(geometry_msgs::Pose start, geometry_msgs::Pose goal_pose);
    void setPlannerOptions(double step_siz, double timeout, Eigen::Vector3d min_bound, Eigen::Vector3d max_bound);

protected:
    std::shared_ptr<ompl::base::SE2StateSpace> space;
    geometry_msgs::Pose pose2Dto3D(double x,double y,double z,double ang);
    float height_z=0.2;
    ompl::base::RealVectorBounds bounds;

};

#endif // PLAN_FSMORE_2D_H
