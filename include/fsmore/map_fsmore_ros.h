#ifndef MAP_FSMORE_ROS_H
#define MAP_FSMORE_ROS_H
#include <fsmore/map_fsmore.h>
#include <fsmore/map_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>

class MapFsmoreROS{
public:
    MapFsmore mapper;
    MapFsmoreROS();
    void LoadSTL(std::string filename,PCTypePtr tree);

protected:
    bool Initialize();
    ros::NodeHandle *n;
    ros::Subscriber force_sub;
    ros::Publisher pub_line,pub_map_pc,pub_obj_pc,pub_oct_map;
    std::string mesh_filename;
    void cb_contforce(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    bool first_ft_cb=true;
    geometry_msgs::Wrench bias_ft;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    inline Eigen::Vector3f toEigen(geometry_msgs::Vector3 in);
    inline Eigen::Affine3f toEigen(geometry_msgs::Transform m);
    visualization_msgs::Marker setupLines(std::string frame_id);
    void AddToMarkerLines(Line l,visualization_msgs::Marker &m);    
    void AddPointsFromPC(PCTypePtr in, OctTypePtr tree_out, PCTypePtr cloud_out, std::map<size_t, Voxel> &map_out);

};

#endif // MAP_FSMORE_ROS_H
