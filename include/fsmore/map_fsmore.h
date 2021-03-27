#ifndef MAP_FSMORE_H
#define MAP_FSMORE_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <fsmore/map_tools.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <map>
#include <memory>


typedef pcl::PointXYZINormal PType;
typedef pcl::PointCloud<PType> PCType;
typedef pcl::octree::OctreePointCloudSearch<PType> OctType;


class Voxel;
class Line;

class Line{
public:
    Line(Eigen::Vector3f start, Eigen::Vector3f end){
        p1=start;
        p2=end;
        dir=p2-p1;
    }

    Line Transform(Eigen::Affine3f T){
        Eigen::Vector3f p1t,p2t;
        p1t=T*p1;
        p2t=T*p2;
        return(Line(p1t,p2t));
    }
    Eigen::Vector3f p1,p2,dir;
    std::vector<float> lkl;
    std::vector<std::shared_ptr<Voxel> > voxels;

protected:
};

class Voxel{
public:
    Voxel(OctType::Ptr tree, pcl::octree::OctreeKey in){
        key=in;
        map=tree;

        //point=tree->keyToCoord(key);
        //p=Eigen::Vector3f(point.x(),point.y(),point.z());
    }
    PType TransformCoord(Eigen::Affine3f T){
        Eigen::Vector3f p_out=T*p;
        PType p2;
        p2.getArray3fMap()=p_out;
        //return( octomap::point3d(p_out.x(),p_out.y(),p_out.z()));
    }

    std::vector<Line*> intersecting;
    Eigen::Vector3f p;
    OctType::Ptr map;
    pcl::octree::OctreeKey key;
    PType point;
    float likelihood;

protected:

};


class MapFsmore{
public:
    MapFsmore();
    bool AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T);
    pcl::PointCloud<pcl::PointXYZI>  getMapPointCloud();
    pcl::PointCloud<pcl::PointXYZI>  getObjectPointCloud();
    OctType::Ptr oct_map,oct_obj;
    PCType::Ptr pc_obj,pc_map;

protected:
    const float line_half_length=1.0f;
    std::vector<Line> lines_map,lines_obj;
    std::map<pcl::octree::OctreeKey,Voxel> map_map,map_obj;

    Line ForceToLine(Eigen::Vector3f F_in, Eigen::Vector3f M_in, Eigen::Vector3f &p1, Eigen::Vector3f &p2,float &k);

    pcl::PointCloud<pcl::PointXYZI> getPointCloud(OctType::Ptr octree);


};

#endif // MAP_FSMORE_H
