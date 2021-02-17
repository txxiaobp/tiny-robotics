#ifndef LINEARWITHPARABOLATRANSITIONPLANNING_H
#define LINEARWITHPARABOLATRANSITIONPLANNING_H


#include "motion_planning.h"

/*
 * 已知起始位置和速度、终止位置和速度，进行基于三次多项式的轨迹规划
 */
class LinearWithParabolaTransitionPlanning : public MotionPlanning
{
public:
    LinearWithParabolaTransitionPlanning(double timeIntervel,
                                         double timeStep,
                                         std::vector<std::pair<double,double>> &posConstraint,
                                         std::vector<std::pair<double,double>> &velConstraint,
                                         std::vector<std::pair<double,double>> &accelConstraint,
                                         double maxAcceleration);
    ~LinearWithParabolaTransitionPlanning() {}

    bool plan(std::vector<std::pair<double,double>> &posVec,
              std::vector<std::pair<double,double>> &velVec,
              std::vector<std::pair<double,double>> &accelVec);
private:
    double calAcceleration(double velStart, double velEnd);
    void calDurationVelocityAndAcceleration(std::vector<double> &timeVec,
                                            std::vector<double> &posVec,
                                            std::vector<double> &velVec,
                                            std::vector<std::pair<double,double>> &accelVec);

private:
    double maxAcceleration;
};

#endif // LINEARWITHPARABOLATRANSITIONPLANNING_H
