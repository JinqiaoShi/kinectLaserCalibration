#ifndef SOLVER_H
#define SOLVER_H
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "spinner.h"
#include "geohelper.h"

template <class Solver,class InputData,class OutputData>
class genericSolver{
public:
    void setInitialTransform(OutputData t){
        _algorithm.setInitialGuess(t);
    }
    void setIterationsNum(int i){
        _algorithm.setIterationNum(i);
    }

    OutputData getOutputData(){return _output;}

    void optimize(InputData i){
        _output = _algorithm.optimize(i);
    }
private:
    InputData _input;
    OutputData _guess;
    OutputData _output;
    Solver _algorithm;
};




#endif
