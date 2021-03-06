#ifndef MAP_FSMORE_ROS_H
#define MAP_FSMORE_ROS_H
#include <fsmore/map_fsmore.h>
#include <fsmore/map_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>
#include <octomap_msgs/GetOctomap.h>

class MapFsmoreROS{
public:
    MapFsmore mapper;
    MapFsmoreROS();
    void LoadSTL(std::string filename,PCTypePtr tree);

protected:
    bool Initialize();
    ros::NodeHandle *n;
    ros::Subscriber force_sub,pose_sub;
    ros::Publisher pub_line,pub_map_pc,pub_obj_pc,pub_oct_map,pub_oct_obj;
    ros::ServiceServer get_map_octree,get_obj_octree;

    std::string mesh_filename,world_frame,object_frame;
    void cb_contforce(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void cb_eepose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool first_ft_cb=true;
    geometry_msgs::Wrench bias_ft;
    geometry_msgs::PoseStamped last_pose;
    ros::Time last_hit;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    inline Eigen::Vector3f toEigen(geometry_msgs::Vector3 in);
    inline Eigen::Affine3f toEigen(geometry_msgs::Transform m);
    visualization_msgs::Marker setupLines(std::string frame_id);
    void AddToMarkerLines(Line l,visualization_msgs::Marker &m);    
    void AddPointsFromPC(PCTypePtr in, OctTypePtr tree_out, PCTypePtr cloud_out, std::map<size_t, Voxel> &map_out);
    void PublishOctrees(OctTypePtr octo, ros::Publisher pub, std::string frame);
    void PublishPointCloud(pcl::PointCloud<pcl::PointXYZI> pc,ros::Publisher pub,std::string frame);
    bool getOctomap(octomap_msgs::GetOctomap::Request  &req, octomap_msgs::GetOctomap::Response &res, OctTypePtr plan, std::string frame);


};

#endif // MAP_FSMORE_ROS_H
