#ifndef ROBOT_H
#define ROBOT_H

#include "link.h"
#include <vector>

typedef std::vector<std::pair<double, std::pair<double, double>>> PlanVec;
typedef std::vector<std::pair<double, double>> ConVec; // constraint vector of planning

class Robot
{
public:
    Robot();
    ~Robot() {}
    void addLink(const Link &link);
    Vector getEndPos() const;
    Vector getEndVel() const;

    void setJointIncrement(std::vector<double> incrementVec);
    void setJointVelocity(std::vector<double> velVec);
    std::vector<double> getJointVelocity() const;
    Vector getJacobian() const;

    /*
     * 逆向运动学，该机器人应满足Pieper准则，即：
     * 1. 三个相邻关节轴线交于一点
     * 2. 三个相邻关节轴线相互平行
     */
    virtual std::vector<double> inverseKinematics(Matrix &endPos) = 0;


    PlanVec motionPlan(ConVec &posConVec, ConVec &velConVec);

    static Vector getDiffMatrix(const Matrix &matrix, Joint_E jointType);

protected:
    std::vector<Link> linkVec;
};

#endif // ROBOT_H
