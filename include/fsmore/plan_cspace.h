#ifndef PLAN_CSPACE_H
#define PLAN_CSPACE_H
#include <fsmore/plan_fsmore.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>


class Plan_CSpace : public PlanFsmore
{
public:
    Plan_CSpace();
    Plan_CSpace(ros::NodeHandle *n);
    void setBounds(Eigen::Vector3d min,Eigen::Vector3d max);
    bool isStateValid(const ompl::base::State *state);
    void setStartAndGoal(geometry_msgs::Pose start,geometry_msgs::Pose goal);
    void contact_cb(const geometry_msgs::WrenchStamped &msg);
    void pose_cb(const geometry_msgs::PoseStamped &msg);
    geometry_msgs::Pose2D pose3Dto2D(geometry_msgs::Pose in);
    double diff_pose(geometry_msgs::Pose2D p1, const ompl::base::SE2StateSpace::StateType *p2);
    void setupPlanner();
    ompl::base::PlannerStatus getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal, geometry_msgs::PoseArray &solution, geometry_msgs::PoseArray &vertexes);
    void setPlannerOptions(double step_siz, double timeout, Eigen::Vector3d min_bound, Eigen::Vector3d max_bound);
    bool isValid(geometry_msgs::Pose pose);

    geometry_msgs::PoseStamped current_pose;
    std::vector<geometry_msgs::Pose2D> occupied_poses;
    std::vector<geometry_msgs::Pose> occupied_poses3D;
    ros::Subscriber sub1,sub2;

    ros::NodeHandle* nh;
    ompl::base::RealVectorBounds bounds;
    std::shared_ptr<ompl::base::SE2StateSpace> space;
    float height_z=0.2;
    geometry_msgs::Pose pose2Dto3D(double x,double y,double z,double ang);

};

#endif // PLAN_CSPACE_H
