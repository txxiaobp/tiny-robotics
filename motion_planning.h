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
    virtual void plan(std::vector<std::pair<double,double>> &posVec,
                      std::vector<std::pair<double,double>> &velVec,
                      std::vector<std::pair<double,double>> &accelVec) = 0;

protected:
    double timeIntervel;
    double timeStep;
    std::vector<std::pair<double,double>> &posConstraint;
    std::vector<std::pair<double,double>> &velConstraint;
    std::vector<std::pair<double,double>> &accelConstraint;
};

#endif // MOTION_PLANNING_H
