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

typedef pcl::PointXYZINormal PType;
typedef pcl::PointCloud<PType> PCType;
typedef pcl::octree::OctreePointCloudSearch<PType> OctType;
typedef pcl::octree::OctreeKey MapKey;

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
    Voxel(OctType::Ptr tree, PType point_in, size_t key_in){
        key=key_in;
        map=tree;
        //point=point_in;
        std::vector<int> p_inside;
        map->voxelSearch(point_in,p_inside);
        point=(PType*) &(map->getInputCloud()->at(p_inside[0]));
        //likelihood=&(map->getInputCloud()->at(p_inside[0]).intensity);
        likelihood=(float*) &(point->intensity);
        p=Eigen::Vector3f(point->x,point->y,point->z);
    }
    PType TransformCoord(Eigen::Affine3f T){
        Eigen::Vector3f p_out=T*p;
        PType p2;
        p2.x=p_out.x();
        p2.y=p_out.y();
        p2.z=p_out.z();
        return(p2);
        //p2.getArray3fMap()=p_out;
        //return( octomap::point3d(p_out.x(),p_out.y(),p_out.z()));
    }
    void setLikelihood(float in){
        point->intensity=in;
    }

    std::vector<Line*> intersecting;
    Eigen::Vector3f p;
    OctType::Ptr map;
    size_t key;
    PType *point;
    float *likelihood;

protected:

};


class MapFsmore{
public:
    MapFsmore();
    bool AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T);
    PCType getMapPointCloud();
    PCType getObjectPointCloud();
    OctType::Ptr oct_map,oct_obj;
    PCType::Ptr pc_obj,pc_map;
    size_t KeyHasher(MapKey key_arg);
    std::map<size_t,Voxel> map_map,map_obj;
protected:
    const float line_half_length=1.0f;
    const float line_res=0.01f;
    std::vector<Line> lines_map,lines_obj;

    Line ForceToLine(Eigen::Vector3f F_in, Eigen::Vector3f M_in, Eigen::Vector3f &p1, Eigen::Vector3f &p2,float &k);

    pcl::PointCloud<pcl::PointXYZI> getPointCloud(OctType::Ptr octree);


};

#endif // MAP_FSMORE_H
