#ifndef LSSOLVER_H
#define LSSOLVER_H
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "solver.h"

namespace Malcom {
class lsSolver;
}

class Malcom::lsSolver : Malcom::solver
{
public:
    void getCorrespondaces(std::vector<class pointAssoc*> &pA);
    void getInitialTransform(Eigen::Vector3d t);
    void optimize(int i);

private:
    Eigen::Vector3d t;
    std::vector<class pointAssoc*> pA;
};

#endif
