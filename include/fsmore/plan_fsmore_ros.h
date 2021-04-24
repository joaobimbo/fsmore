#ifndef PLAN_FSMORE_ROS_H
#define PLAN_FSMORE_ROS_H
#include <ros/ros.h>
//#include <fsmore/plan_fsmore.h>
#include <fsmore/plan_fsmore_2d.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <boost/bind.hpp>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <fsmore/map_fsmore.h>

class PlanFsmoreROS
{
public:
    PlanFsmoreROS();
    bool planPath(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res, PlanFsmore_2D *planner);

protected:
    ros::NodeHandle* n;
    ros::ServiceServer plan_service;
    std::string mesh_filename,world_frame;
    ros::Publisher pub_plan_markers;
//    ros::Subscriber sub_oct_map,sub_oct_obj;
    ros::Publisher pub_map;
    ros::Publisher pub_obj;


    ros::ServiceClient srv_oct_map,srv_oct_obj;

    void initializePlanner();
    PlanFsmore_2D *planner;

    void cb_oct_map(const octomap_msgs::Octomap::ConstPtr& msg);
    void cb_oct_obj(const octomap_msgs::Octomap::ConstPtr& msg);
    void publishMarkerArray(std::vector<geometry_msgs::Pose> poses);

};

#endif // PLAN_FSMORE_ROS_H
