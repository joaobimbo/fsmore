#ifndef MAP_FSMORE_H
#define MAP_FSMORE_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <fsmore/map_tools.h>
#include <octomap/octomap.h>

class Line{
public:
    Line(Eigen::Vector3f start, Eigen::Vector3f end){
        p1=start;
        p2=end;
        dir=p2-p1;
    }

    Line Transform(Eigen::Affine3f T){
        Eigen::Vector3f p1t,p2t;
        ///TODO:  Do the transform
        return(Line(p1t,p2t));
    }
    Eigen::Vector3f p1,p2,dir;
    std::vector<float> lkl;

protected:
};

class MapFsmore{
public:
    MapFsmore();
    bool AddLine(Eigen::Vector3f F, Eigen::Vector3f M, Eigen::Affine3f T);
    template <class PType> std::vector<PType> getMapPointCloud();

protected:
    const float line_half_length=1.0f;
    std::vector<Line> lines;
    octomap::OcTree *map;

    Line ForceToLine(Eigen::Vector3f F_in, Eigen::Vector3f M_in, Eigen::Vector3f &p1, Eigen::Vector3f &p2,float &k);

    template <class PType> std::vector<PType> getPointCloud(octomap::OcTree* octree);



};

#endif // MAP_FSMORE_H
