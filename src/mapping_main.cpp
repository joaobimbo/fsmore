#include <ros/ros.h>
#include <stdio.h>
#include <fsmore/map_fsmore_ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_fsmore_node");
    MapFsmoreROS map_ros;
    ros::spin();
    return(0);
}
