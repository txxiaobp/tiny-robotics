#ifndef QUINTICPOLYNOMIAL_H
#define QUINTICPOLYNOMIAL_H

#include "motion_planning.h"

/*
 * 已知起始位置和终止位置，进行基于三次多项式的轨迹规划
 * 仅适用于起始角度和终止角速度为零的情况
 */
class QuinticPolynomialPlanning : public MotionPlanning
{
public:
    QuinticPolynomialPlanning(double timeIntervel,
                              double timeStep,
                              std::vector<std::pair<double,double>> &posConstraint,
                              std::vector<std::pair<double,double>> &velConstraint,
                              std::vector<std::pair<double,double>> &accelConstraint);
    ~QuinticPolynomialPlanning() {}

    void plan(std::vector<std::pair<double,double>> &posVec,
              std::vector<std::pair<double,double>> &velVec,
              std::vector<std::pair<double,double>> &accelVec);
};

#endif // QUINTICPOLYNOMIAL_H
