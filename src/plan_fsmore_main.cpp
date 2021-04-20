#include <ros/ros.h>
#include <stdio.h>
#include <fsmore/plan_fsmore_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_fsmore_node");
    PlanFsmoreROS plan_ros;
    ros::spin();
    return(0);
}
