#ifndef SOLVER_H
#define SOLVER_H
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "spinner.h"

class pointAssoc;

namespace Malcom {
    class solver;
}

class Malcom::solver
{
public:
    virtual void getCorrespondaces(std::vector<class pointAssoc*> &pA)=0;
    virtual void getInitialTransform(Eigen::Vector3d t)=0;
    virtual void optimize(int i)=0;

};

#endif
