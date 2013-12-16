#ifndef GEOHELPER_H
#define GEOHELPER_H
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <vector>
#include <ros/ros.h>
#include <ros/message.h>
#include "geometry_msgs/Point32.h"

namespace Malcom {
    class geometry;
}

class Malcom::geometry
{
public:
    static Eigen::Vector3d t2v( Eigen::Matrix3d x);
    static Eigen::Matrix3d v2t( Eigen::Vector3d x);
};

typedef std::pair<geometry_msgs::Point32,geometry_msgs::Point32 > pointpair;
typedef std::pair<float,float> laserc;
typedef std::pair<laserc,laserc> laserpair;

typedef std::vector<pointpair > pointpairVec;
typedef std::vector<laserpair > laserpairVec;

#endif
