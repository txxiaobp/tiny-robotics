#include "cubic_polynomial_planning.h"
#include <cassert>
#include <cmath>
#include <iostream>

CubicPolynomialPlanning::CubicPolynomialPlanning(double timeIntervel,
                                                 double timeStep,
                                                 std::vector<std::pair<double,double>> &posConstraint,
                                                 std::vector<std::pair<double,double>> &velConstraint,
                                                 std::vector<std::pair<double,double>> &accelConstraint)
    : MotionPlanning(timeIntervel, timeStep, posConstraint, velConstraint, accelConstraint)
{
    assert(posConstraint.size() == 2);
    assert(velConstraint.size() == 2);
    assert(accelConstraint.size() == 2);
    assert(posConstraint[0].first == 0);
    assert(posConstraint[1].first == timeIntervel);
    assert(velConstraint[0].first == 0);
    assert(velConstraint[1].first == timeIntervel);
    assert(accelConstraint[0].first == 0);
    assert(accelConstraint[1].first == timeIntervel);
}

bool CubicPolynomialPlanning::plan(std::vector<std::pair<double,double>> &posVec,
                                   std::vector<std::pair<double,double>> &velVec,
                                   std::vector<std::pair<double,double>> &accelVec)
{
    double theta0 = posConstraint[0].second;
    double thetaf = posConstraint[1].second;
    double dTheta0 = velConstraint[0].second;
    double dThetaf = velConstraint[1].second;

    double a0 = theta0;
    double a1 = dTheta0;

    double a2 = 3 * (thetaf - theta0) / pow(timeIntervel, 2) - 2 * dTheta0 / timeIntervel - dThetaf / timeIntervel;
    double a3 = -2 * (thetaf - theta0) / pow(timeIntervel, 3) + (dTheta0 + dThetaf) / pow(timeIntervel, 2);

    posVec.clear();
    velVec.clear();
    accelVec.clear();

    int timeCount = ceil(timeIntervel / timeStep) + 1;
    double time = 0;
    for (int i = 0; i < timeCount; i++)
    {
        posVec.push_back(std::make_pair(time, a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3)));
        velVec.push_back(std::make_pair(time, a1 + 2 * a2 * time + 3 * a3 * pow(time, 2)));
        accelVec.push_back(std::make_pair(time, 2 * a2 + 6 * a3 * time));
        time = std::min(time + timeStep, timeIntervel);
    }

    return true;
}
