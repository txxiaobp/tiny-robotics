#include "quintic_polynomial_planning.h"
#include <cassert>
#include <cmath>
#include <iostream>

QuinticPolynomialPlanning::QuinticPolynomialPlanning(double timeIntervel,
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

void QuinticPolynomialPlanning::plan(std::vector<std::pair<double,double>> &posVec,
                                   std::vector<std::pair<double,double>> &velVec,
                                   std::vector<std::pair<double,double>> &accelVec)
{
    double theta0 = posConstraint[0].second;
    double thetaf = posConstraint[1].second;
    double dTheta0 = velConstraint[0].second;
    double dThetaf = velConstraint[1].second;
    double ddTheta0 = accelConstraint[0].second;
    double ddThetaf = accelConstraint[1].second;

    double a0 = theta0;
    double a1 = dTheta0;
    double a2 = ddTheta0 / 2;
    double a3 = (20 * thetaf - 20 * theta0 - (8 * dThetaf + 12 * dTheta0) * timeIntervel - (3 * ddTheta0 - ddThetaf) * pow(timeIntervel, 2))
                / (2 * pow(timeIntervel, 3));

    double a4 = (30 * theta0 - 30 * thetaf + (14 * dThetaf + 16 * dTheta0) * timeIntervel + (3 * ddTheta0 - 2 * ddThetaf) * pow(timeIntervel, 2))
                / (2 * pow(timeIntervel, 4));

    double a5 = (12 * thetaf - 12 * theta0 - (6 * dThetaf + 6 * dTheta0) * timeIntervel - (ddTheta0 - ddThetaf) * pow(timeIntervel, 2))
                / (2 * pow(timeIntervel, 5));

    posVec.clear();
    velVec.clear();
    accelVec.clear();

    int timeCount = ceil(timeIntervel / timeStep) + 1;
    double time = 0;
    for (int i = 0; i < timeCount; i++)
    {
        posVec.push_back( std::make_pair(time, a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3) + a4 * pow(time, 4) + a5 * pow(time, 5)) );
        velVec.push_back(std::make_pair(time, a1 + 2 * a2 * time + 3 * a3 * pow(time, 2) + 4 * a4 * pow(time, 5) + 5 * a5 * pow(time, 4)));
        accelVec.push_back(std::make_pair(time, 2 * a2 + 6 * a3 * time + 12 * a4 * pow(time, 2) + 20 * a5 * pow(time, 3)));
        time = std::min(time + timeStep, timeIntervel);
    }
}
