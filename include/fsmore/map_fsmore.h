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
#include <boost/functional/hash.hpp>
#include <pcl/filters/voxel_grid.h>
#include <octomap/OcTree.h>
//#include <fcl/octree.h>

//typedef pcl::PointXYZINormal PType;
//typedef pcl::PointCloud<PType> PCType;
//typedef pcl::octree::OctreePointCloudSearch<PType> OctType;
//typedef pcl::octree::OctreeKey MapKey;
//typedef Eigen::Vector3i KeyType;
//typedef pcl::VoxelGrid<PType> OctType;
typedef octomap::OcTree OctType;
typedef std::shared_ptr<octomap::OcTree> OctTypePtr;
typedef octomap::point3d PType;
typedef octomap::Pointcloud PCType;
typedef std::shared_ptr<octomap::Pointcloud> PCTypePtr;
typedef octomap::OcTreeKey KeyType;

class Voxel;
class Line;

class Line{
public:
    Line(Eigen::Vector3f start, Eigen::Vector3f end){
        p1=start;
        p2=end;
        dir=(p2-p1).normalized();
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
    Voxel(OctTypePtr tree, PType point_in, size_t key_in){
        //key=key_in;
        map=tree;
        key=map->coordToKey(point_in);

        KeyType key_p=map->coordToKey(point_in);
        point = map->keyToCoord(key_p);

        p=Eigen::Vector3f(point.x(),point.y(),point.z());
    }
    PType TransformCoord(Eigen::Affine3f T){
        Eigen::Vector3f p_out=T*p;
        PType p2;
        p2.x()=p_out.x();
        p2.y()=p_out.y();
        p2.z()=p_out.z();
        return(p2);
    }
    void setLikelihood(float in){
        map->setNodeValue(key,1/(1+exp(-in)));
    }
    float getLikelihood(){
        return(map->search(key)->getOccupancy());
    }

    std::vector<Line*> intersecting;
    Eigen::Vector3f p;
    OctTypePtr map;
    KeyType key;
    PType point;
    float *likelihood;

protected:

};


class MapFsmore{
public:
    MapFsmore();
    bool AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T);
    pcl::PointCloud<pcl::PointXYZI> getMapPointCloud();
    pcl::PointCloud<pcl::PointXYZI> getObjectPointCloud();
    OctTypePtr oct_map,oct_obj;
    PCTypePtr pc_obj,pc_map;
    size_t KeyHasher(KeyType key_arg);
    std::map<size_t,Voxel> map_map,map_obj;
protected:
    const float line_half_length=1.0f;
    const float line_res=0.01f;
    std::vector<Line> lines_map,lines_obj;

    Line ForceToLine(Eigen::Vector3f F_in, Eigen::Vector3f M_in, Eigen::Vector3f &p1, Eigen::Vector3f &p2,float &k);

    pcl::PointCloud<pcl::PointXYZI> getPointCloud(OctTypePtr octree);


};

#endif // MAP_FSMORE_H
