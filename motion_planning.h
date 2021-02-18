#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H

#include <vector>

class MotionPlanning
{
public:
    MotionPlanning(double timeIntervel,
                   double timeStep,
                   std::vector<std::pair<double,double>> &posConstraint,
                   std::vector<std::pair<double,double>> &velConstraint,
                   std::vector<std::pair<double,double>> &accelConstraint);
    virtual ~MotionPlanning();
    virtual bool plan(std::vector<std::pair<double,double>> &posVec,
                      std::vector<std::pair<double,double>> &velVec,
                      std::vector<std::pair<double,double>> &accelVec) = 0;

protected:
    double timeIntervel;
    double timeStep;
    std::vector<std::pair<double,double>> &posConstraint; // time, pos
    std::vector<std::pair<double,double>> &velConstraint; // time, velocity
    std::vector<std::pair<double,double>> &accelConstraint; // time, accelerate
};

#endif // MOTION_PLANNING_H
