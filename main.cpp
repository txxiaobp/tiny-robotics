#include <iostream>
#include <cmath>
#include "pub_include.h"
#include "link.h"
#include "robot.h"
#include "six_dof_robot.h"
#include "scara.h"
#include "cubic_polynomial_planning.h"
#include "quintic_polynomial_planning.h"
#include "csv_operate.h"

int main()
{
//    Link link0(JOINT_NONE, 0, 0, 0, 0);  // 固定于地面的虚杆
//    link0.setFix(true);
//    Link link1(JOINT_REVOLUTE, 0, M_PI / 4, 0, 0);
//    Link link2(JOINT_NONE, 0, 0, 10, 0);  // 原点位于末端点的虚杆

//    SixDOFRobot robot;
//    robot.addLink(link0);
//    robot.addLink(link1);
//    robot.addLink(link2);

//    std::vector<double> vec{1};
//    robot.setJointVelocity(vec);

//    Vector endPos = robot.getEndPos();
//    //endPos.showMatrix();

//    Vector endVel = robot.getEndVel();
//    endVel.showMatrix();


    double timeInterval = 15;
    double timeStep = 0.1;
    double timeStart = 0;
    double timeEnd = timeInterval;
    double posStart = 15;
    double posEnd = 75;
    double velStart = 0;
    double velEnd = 0;
    double accelStart = 0;
    double accelEnd = 0;

    std::vector<std::pair<double,double>> posConstraint{std::make_pair(timeStart, posStart), std::make_pair(timeEnd, posEnd)};
    std::vector<std::pair<double,double>> velConstraint{std::make_pair(timeStart, velStart), std::make_pair(timeEnd, velEnd)};
    std::vector<std::pair<double,double>> accelConstraint{std::make_pair(timeStart, accelStart), std::make_pair(timeEnd, accelEnd)};

    CubicPolynomialPlanning cubic(timeInterval, timeStep, posConstraint, velConstraint, accelConstraint);
    QuinticPolynomialPlanning quintic(timeInterval, timeStep, posConstraint, velConstraint, accelConstraint);

    std::vector<std::pair<double,double>> posVec;
    std::vector<std::pair<double,double>> velVec;
    std::vector<std::pair<double,double>> accelVec;
    cubic.plan(posVec, velVec, accelVec);
    //quintic.plan(posVec, velVec, accelVec);

    CSVOperate().exportToFile(posVec);

    return 0;
}
