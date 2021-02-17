#ifndef CUBICPOLYNOMIALPLANNING_H
#define CUBICPOLYNOMIALPLANNING_H

#include "motion_planning.h"

/*
 * 已知起始位置和速度、终止位置和速度，进行基于三次多项式的轨迹规划
 */
class CubicPolynomialPlanning : public MotionPlanning
{
public:
    CubicPolynomialPlanning(double timeIntervel,
                            double timeStep,
                            std::vector<std::pair<double,double>> &posConstraint,
                            std::vector<std::pair<double,double>> &velConstraint,
                            std::vector<std::pair<double,double>> &accelConstraint);
    ~CubicPolynomialPlanning() {}

    void plan(std::vector<std::pair<double,double>> &posVec,
              std::vector<std::pair<double,double>> &velVec,
              std::vector<std::pair<double,double>> &accelVec);
};

#endif // CUBICPOLYNOMIALPLANNING_H
