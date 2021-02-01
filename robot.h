#ifndef ROBOT_H
#define ROBOT_H

#include "link.h"
#include <vector>

class Robot
{
public:
    Robot();
    void addLink(const Link &link);
    Vector getEndPos() const;
    Vector getEndVel() const;

    void setJointIncrement(std::vector<double> incrementVec);

    /*
     * 逆向运动学，该机器人应满足Pieper准则，即：
     * 1. 三个相邻关节轴线交于一点
     * 2. 三个相邻关节轴线相互平行
     */
    virtual std::vector<double> inverseKinematics(Vector &endPos) = 0;

private:
    std::vector<Link> linkVec;
};

#endif // ROBOT_H
