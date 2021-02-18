#include <iostream>
#include <cmath>
#include "pub_include.h"
#include "link.h"
#include "robot.h"
#include "six_dof_robot.h"
#include "scara.h"
#include "cubic_polynomial_planning.h"
#include "quintic_polynomial_planning.h"
#include "linear_with_parabola_transition_planning.h"
#include "cubic_spline_planning.h"
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

    double d1 = 0.33;
    double d3 = 0.1;
    double l1 = 0.325;
    double l2 = 0.275;
    Link link0(JOINT_NONE, 0, 0, 0, 0);  // 固定于地面的虚杆
    link0.setFix(true);
    Link link1(JOINT_REVOLUTE, 0, 0, d1, 0);
    Link link2(JOINT_REVOLUTE, l1, 0, 0, 0);
    Link link3(JOINT_TRANSLATE, l2, 0, 0, -M_PI);
    Link link4(JOINT_REVOLUTE, 0, 0, d3, 0);

    Scara scara;
    scara.addLink(link0);
    scara.addLink(link1);
    scara.addLink(link2);
    scara.addLink(link3);
    scara.addLink(link4);



//    double timeInterval = 6;
//    double timeStep = 0.1;
//    double timeStart = 0;
//    double timeEnd = timeInterval;
//    double posStart = 15;
//    double posEnd = 75;
//    double velStart = 0;
//    double velEnd = 0;
//    double accelStart = 0;
//    double accelEnd = 0;

//    std::vector<std::pair<double,double>> posConstraint
//        {
//            std::make_pair(0, 10),
//            std::make_pair(2, 35),
//            std::make_pair(3, 25),
//            std::make_pair(6, 10),
//        };
//    std::vector<std::pair<double,double>> velConstraint{
//        std::make_pair(0, 0),
//        std::make_pair(6, 0),
//    };
//    std::vector<std::pair<double,double>> accelConstraint;

//    std::vector<std::pair<double,double>> velConstraint{std::make_pair(timeStart, velStart), std::make_pair(timeEnd, velEnd)};
//    std::vector<std::pair<double,double>> accelConstraint{std::make_pair(timeStart, accelStart), std::make_pair(timeEnd, accelEnd)};

//    CubicPolynomialPlanning cubic(timeInterval, timeStep, posConstraint, velConstraint, accelConstraint);
//    QuinticPolynomialPlanning quintic(timeInterval, timeStep, posConstraint, velConstraint, accelConstraint);

//    std::vector<std::pair<double,double>> posVec;
//    std::vector<std::pair<double,double>> velVec;
//    std::vector<std::pair<double,double>> accelVec;

    //cubic.plan(posVec, velVec, accelVec);
    //quintic.plan(posVec, velVec, accelVec);

//    double maxAcceleration = 50;
//    LinearWithParabolaTransitionPlanning linearPlanning(timeInterval, timeStep, posConstraint, velConstraint, accelConstraint, maxAcceleration);
//    linearPlanning.plan(posVec, velVec, accelVec);


//    CubicSplinePlanning cubicPlan(timeInterval, timeStep, posConstraint, velConstraint, accelConstraint);
//    cubicPlan.plan(posVec, velVec, accelVec);


//    std::cout << posVec.size() << std::endl;
//    for (auto pair : posVec)
//    {
//        std::cout << pair.first << ":  " << pair.second << std::endl;
//    }


//    CSVOperate().exportToFile(posVec);

    return 0;
}
