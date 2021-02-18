#ifndef CUBICSPLINEPLANNING_H
#define CUBICSPLINEPLANNING_H


#include "motion_planning.h"
#include "matrix.h"

/*
 * 已知起始位置和速度、终止位置和速度，进行基于三次多项式的轨迹规划
 */
class CubicSplinePlanning : public MotionPlanning
{
public:
    CubicSplinePlanning(double timeIntervel,
                            double timeStep,
                            std::vector<std::pair<double,double>> &posConstraint,
                            std::vector<std::pair<double,double>> &velConstraint,
                            std::vector<std::pair<double,double>> &accelConstraint);
    ~CubicSplinePlanning() {}

    bool plan(std::vector<std::pair<double,double>> &posVec,
              std::vector<std::pair<double,double>> &velVec,
              std::vector<std::pair<double,double>> &accelVec);

private:
    double calDifferenceQuotient(double y1, double y2, double x1, double x2);
    double calDifferenceQuotient(double y1, double y2, double y3, double x1, double x2, double x3);
    void calRhs(Vector &rhsM);
    void calHVec(std::vector<double> &hVec);
    void calLambdaVec(const std::vector<double> &hVec, std::vector<double> &lambdaVec);
    void calMuVec(const std::vector<double> &hVec, std::vector<double> &muVec);
    void calcCoefficientMatrix(const std::vector<double> &lambdaVec, const std::vector<double> &muVec, Vector &matrix);
};

#endif // CUBICSPLINEPLANNING_H
