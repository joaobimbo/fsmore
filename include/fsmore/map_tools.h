#ifndef MAP_TOOLS_H
#define MAP_TOOLS_H
#include <eigen3/Eigen/Core>

namespace MapTools {
/*
inline Eigen::Vector3f toEigen(float in[3] ){
    Eigen::Vector3f out;
    out.x()=in[0];
    out.y()=in[1];
    out.z()=in[2];
    return(out);
}
*/

template <class VType> double norm(VType in){
  return(sqrt(in.x*in.x+in.y*in.y+in.z*in.z));
}

template <class VType> VType v_minus(VType in1,VType in2){
  VType out;
  out.x=in1.x-in2.x;
  out.y=in1.y-in2.y;
  out.z=in1.z-in2.z;
  return(out);
}


}

#endif // MAP_TOOLS_H
