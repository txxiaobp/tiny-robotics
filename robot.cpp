#include "robot.h"
#include "transform_matrix.h"
#include <cassert>

Robot::Robot()
{
    linkVec.clear();
}

void Robot::addLink(const Link &link)
{
    linkVec.push_back(link);
}

/* 计算末端位置 */
Vector Robot::getEndPos() const
{
    assert(!linkVec.empty());
    assert(linkVec[0].isFixed()); // 第一个连杆需要固定在地面

    std::vector<double> vec(4, 1);
    Matrix dhMatrix(vec, 4);

    for (auto vecIter = linkVec.begin() + 1; vecIter < linkVec.end(); ++vecIter)
    {
        // DH建模
        dhMatrix *= TransformMatrix::rotate(AXIS_Z, vecIter->getXAngle());
        dhMatrix *= TransformMatrix::translate(AXIS_Z, vecIter->getXBias());
        dhMatrix *= TransformMatrix::rotate(AXIS_X, vecIter->getZAngle());
        dhMatrix *= TransformMatrix::translate(AXIS_X, vecIter->getZBias());
    }
    return dhMatrix.getSubMatrix(0, 3, 3, 4);
}

/* 计算末端速度 */
Vector Robot::getEndVel() const
{

}

/* 设置机器人各关节的位置 */
void Robot::setJointIncrement(std::vector<double> incrementVec)
{
    assert(incrementVec.size() == linkVec.size() - 2); // 去掉头尾的虚杆
    for (decltype (linkVec.size()) i = 1; i < linkVec.size() - 1; i++)
    {
        linkVec[i].move(incrementVec[i - 1]);
    }
}

/*
 * 逆向运动学，该机器人应满足Pieper准则，即：
 * 1. 三个相邻关节轴线交于一点
 * 2. 三个相邻关节轴线相互平行
 */
std::vector<double> Robot::inverseKinematics(Vector &endPos)
{

}

