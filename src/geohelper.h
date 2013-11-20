#ifndef GEOHELPER_H
#define GEOHELPER_H
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace Malcom {
    class geometry;
}

class Malcom::geometry
{
public:
    static Eigen::Vector3d t2v( Eigen::Matrix3d x);
    static Eigen::Matrix3d v2t( Eigen::Vector3d x);
};

#endif
