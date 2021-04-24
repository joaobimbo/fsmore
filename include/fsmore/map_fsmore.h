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

#include <time.h>

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

class Line : public std::enable_shared_from_this<Line>{
public:
    Line(){

    }
    Line(Eigen::Vector3f start, Eigen::Vector3f end){
        p1=start;
        p2=end;
        dir=(p2-p1).normalized();
        time(&timestamp);
    }
    Line(const Line &other){
       *this=other;
       for(int i=0;i<other.voxels.size();i++) this->voxels.at(i)=(other.voxels.at(i));
    }

    Line Transform(Eigen::Affine3f T){
        Eigen::Vector3f p1t,p2t;
        p1t=T*p1;
        p2t=T*p2;
        return(Line(p1t,p2t));
    }
    float InnerProd(Line in){
        return(dir.dot(in.dir));
    }

    Eigen::Vector3f p1,p2,dir;
    std::vector<float> prob_contact;
    std::vector<Voxel*> voxels;
    time_t timestamp;

protected:
};

class Voxel : public std::enable_shared_from_this<Voxel> {
public:
    Voxel(OctTypePtr tree, PType point_in, size_t key_in){
        //key=key_in;
        map=tree;
        key=map->coordToKey(point_in);
        point = map->keyToCoord(key);
        p=Eigen::Vector3f(point.x(),point.y(),point.z());
        likelihood=0.0f;
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
        map->setNodeValue(key,log(in/(1-in)));        
        //OctType::NodeType *n=map->search(key);
        //n->setValue(in);
        likelihood=in;
        return;
    }
    float getLikelihood(){
        return(likelihood);
       // OctType::NodeType *n=map->search(key);
       // if(n!=nullptr) return(n->getOccupancy());//return(n->getLogOdds());
       // else return(0.5f);
    }
    bool LineExists(Line in){
        float max_dotprod=0;
        for (int i=0;i<intersecting.size();i++){
            float mag_dprod=fabs(intersecting.at(i)->InnerProd(in));
            if(mag_dprod > max_dotprod) max_dotprod=mag_dprod;
        }
        if(max_dotprod>0.95) return(true);
        else return(false);
    }

    //std::vector< std::shared_ptr<Line> > intersecting;
    std::vector<Line*> intersecting;
    Eigen::Vector3f p;
    OctTypePtr map;
    KeyType key;
    PType point;
    float likelihood;
    time_t timestamp;
protected:

};


class MapFsmore{
public:
    MapFsmore();
    bool AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T, Line &l_map, Line &l_obj);
    bool AddEmpty(Eigen::Affine3f T);
    pcl::PointCloud<pcl::PointXYZI> getMapPointCloud();
    pcl::PointCloud<pcl::PointXYZI> getObjectPointCloud();
    OctTypePtr oct_map,oct_obj;
    PCTypePtr pc_obj,pc_map;
    size_t KeyHasher(KeyType key_arg);
    std::map<size_t,Voxel> map_map,map_obj;
    void ComputeProbabilities();
    void CleanupLines();
    double decay_time = 60.0;
protected:
    const float line_half_length=1.0f;
    const float line_res=0.01f;

    const float same_line_tol=0.999;
    std::vector<Line> lines_map,lines_obj;

    Line ForceToLine(Eigen::Vector3f F_in, Eigen::Vector3f M_in, Eigen::Vector3f &p1, Eigen::Vector3f &p2,float &k);

    pcl::PointCloud<pcl::PointXYZI> getPointCloud(OctTypePtr octree);
    bool LineExists(std::vector<Line> &lines, Line &l_in);
    void NormalizeLine(Line in);
    void DeleteLine(OctTypePtr oct, std::map<size_t, Voxel> &map, std::vector<Line> &lines, size_t line_nr, bool keep_maxlik);
    void UpdateProbs(Line *line, OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T, int depth);
    void UpdateFree(OctTypePtr &oct, OctTypePtr &other_oct,  std::map<size_t,Voxel> &map,  std::map<size_t,Voxel> &other_map, Eigen::Affine3f T);
};

#endif // MAP_FSMORE_H
