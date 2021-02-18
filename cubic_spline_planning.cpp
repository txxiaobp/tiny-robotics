#include "cubic_spline_planning.h"
#include <cassert>
#include <cmath>
#include <iostream>

CubicSplinePlanning::CubicSplinePlanning(double timeIntervel,
                                                 double timeStep,
                                                 std::vector<std::pair<double,double>> &posConstraint,
                                                 std::vector<std::pair<double,double>> &velConstraint,
                                                 std::vector<std::pair<double,double>> &accelConstraint)
    : MotionPlanning(timeIntervel, timeStep, posConstraint, velConstraint, accelConstraint)
{
    assert(posConstraint.size() >= 2);
    assert(velConstraint.size() == 2);

    for (auto pair : posConstraint)
    {
        assert(pair.first >= 0 && pair.first <= timeIntervel);
    }
    assert(velConstraint[0].first == 0);
    assert(velConstraint[1].first == timeIntervel);
}

bool CubicSplinePlanning::plan(std::vector<std::pair<double,double>> &posVec,
                                   std::vector<std::pair<double,double>> &velVec,
                                   std::vector<std::pair<double,double>> &accelVec)
{
    posVec.clear();
    velVec.clear();
    accelVec.clear();

    Vector rhsM;
    Matrix coeMatrix;
    std::vector<double> hVec, lambdaVec, muVec;
    calRhs(rhsM);
    calHVec(hVec);
    calLambdaVec(hVec, lambdaVec);
    calMuVec(hVec, muVec);
    calcCoefficientMatrix(lambdaVec, muVec, coeMatrix);

    Vector M = coeMatrix / rhsM;

    int timeCount = ceil(timeIntervel / timeStep) + 1;
    double time = 0;
    int index = 0;
    for (int i = 0; i < timeCount; i++, time += timeStep)
    {
        if (time >= posConstraint[index + 1].first)
        {
            index++;
        }

        double mi = M[index];
        double mip = M[index + 1];
        double hip = hVec[index + 1];
        double xi = posConstraint[index].first;
        double xip = posConstraint[index + 1].first;
        double fi = posConstraint[index].second;
        double fip = posConstraint[index + 1].second;

        double s = mi * pow(xip - time, 3) / 6 / hip +
                mip * pow(time - xi, 3) / 6 / hip +
                (fi - mi * pow(hip, 2) / 6) * (xip - time) / hip +
                (fip - mip * pow(hip, 2) / 6) * (time - xi) / hip;

        double ds = -mi * pow(xip - time, 2) / 2 / hip +
                mip * pow(time - xi, 2) / 2 / hip +
                (fip - fi) / hip -
                (mip - mi) * hip / 6;

        double dds = mi * (xip - time) / hip + mip * (time - xi) / hip;

        posVec.push_back(std::make_pair(time, s));
        velVec.push_back(std::make_pair(time, ds));
        accelVec.push_back(std::make_pair(time, dds));
    }

    return true;
}

void CubicSplinePlanning::calcCoefficientMatrix(const std::vector<double> &lambdaVec, const std::vector<double> &muVec, Vector &matrix)
{
    int size = static_cast<int>(lambdaVec.size());
    matrix = Matrix(size, size);
    matrix.setValue(0, 0, 2);
    matrix.setValue(0, 1, 1);
    matrix.setValue(size - 1, size - 2, 1);
    matrix.setValue(size - 1, size - 1, 2);

    for (int i = 1; i < size - 1; i++)
    {
        matrix.setValue(i, i - 1, lambdaVec[i]);
        matrix.setValue(i, i, 2);
        matrix.setValue(i, i + 1, muVec[i]);
    }
}

void CubicSplinePlanning::calRhs(Vector &rhsM)
{
    int size = static_cast<int>(posConstraint.size());
    rhsM = Vector(size, 1);
    double diffQuot0 = calDifferenceQuotient(posConstraint[0].second, posConstraint[1].second, posConstraint[0].first, posConstraint[1].first);
    double m0 = velConstraint[0].second;
    double h1 = posConstraint[1].first - posConstraint[0].first;
    rhsM[0] = (diffQuot0 - m0) / h1;

    double diffQuotn = calDifferenceQuotient(posConstraint[size - 2].second, posConstraint[size - 1].second, posConstraint[size - 2].first, posConstraint[size - 1].first);
    double mn = velConstraint.back().second;
    double hn = posConstraint[size - 1].first - posConstraint[size - 2].first;
    rhsM[size - 1] = (mn - diffQuotn) / hn;

    for (int i = 1; i < size - 1; i++)
    {
        rhsM[i] = calDifferenceQuotient(posConstraint[i - 1].second, posConstraint[i].second, posConstraint[i + 1].second, posConstraint[i - 1].first, posConstraint[i].first, posConstraint[i + 1].first);
    }

}

double CubicSplinePlanning::calDifferenceQuotient(double y1, double y2, double x1, double x2)
{
    assert(x1 != x2);
    return (y2 - y1) / (x2 - x1);
}

double CubicSplinePlanning::calDifferenceQuotient(double y1, double y2, double y3, double x1, double x2, double x3)
{
    return (calDifferenceQuotient(y2, y3, x2, x3) - calDifferenceQuotient(y1, y2, x1, x2)) / (x3 - x1);
}

void CubicSplinePlanning::calHVec(std::vector<double> &hVec)
{
    int size = static_cast<int>(posConstraint.size());
    hVec = std::vector<double>(size, 0);
    for (int i = 1; i < size; i++)
    {
        hVec[i] = posConstraint[i].first - posConstraint[i - 1].first;
    }
}

void CubicSplinePlanning::calLambdaVec(const std::vector<double> &hVec, std::vector<double> &lambdaVec)
{
    int size = static_cast<int>(hVec.size());
    lambdaVec = std::vector<double>(size, 0);
    for (int i = 1; i < size - 1; i++)
    {
        lambdaVec[i] = hVec[i] / (hVec[i] + hVec[i + 1]);
    }
}

void CubicSplinePlanning::calMuVec(const std::vector<double> &hVec, std::vector<double> &muVec)
{
    int size = static_cast<int>(hVec.size());
    muVec = std::vector<double>(size, 0);
    for (int i = 1; i < size - 1; i++)
    {
        muVec[i] = hVec[i + 1] / (hVec[i] + hVec[i + 1]);
    }
}

