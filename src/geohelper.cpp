#include "geohelper.h"

 Eigen::Matrix3d Malcom::geometry::v2t( Eigen::Vector3d x)
{
    Eigen::Matrix3d A(3,3);
    double c=cos(x(2));
    double s=sin(x(2));
    A<<c,-s,x(0),s,c,x(1),0,0,1;
    return A;
}

 Eigen::Vector3d Malcom::geometry::t2v( Eigen::Matrix3d x)
{
    Eigen::Vector3d v;
    v.setZero();
    v(0)=x(0,2);
    v(1)=x(1,2);
    v(2)=atan2(x(1,0),x(0,0));
    return v;
}
