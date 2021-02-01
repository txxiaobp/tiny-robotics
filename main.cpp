#include <iostream>
#include <cmath>
#include "pub_include.h"
#include "link.h"
#include "robot.h"
#include "six_dof_robot.h"
#include "scara.h"


int main()
{
    Link link0(JOINT_NONE, 0, 0, 0, 0);  // 固定于地面的虚杆
    link0.setFix(true);
    Link link1(JOINT_REVOLUTE, 0, M_PI / 4, 0, 0);
    Link link2(JOINT_NONE, 0, 0, 10, 0);  // 原点位于末端点的虚杆

    SixDOFRobot robot;
    robot.addLink(link0);
    robot.addLink(link1);
    robot.addLink(link2);

    std::vector<double> vec{1};
    robot.setJointVelocity(vec);

    Vector endPos = robot.getEndPos();
    //endPos.showMatrix();

    Vector endVel = robot.getEndVel();
    endVel.showMatrix();
    return 0;
}
