#include "robot.h"
#include "transform_matrix.h"
#include <cassert>
#include <iostream>

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
        dhMatrix *= vecIter->getTransMatrix();
    }
    return dhMatrix.getSubMatrix(0, 3, 3, 4);
}

/* 计算末端速度 */
Vector Robot::getEndVel() const
{
    Matrix jacobian = getJacobian();
    std::vector<double> velVec = getJointVelocity();


    Vector velVector(velVec);
    return jacobian * velVector;
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

/* 设置机器人各关节的速度 */
void Robot::setJointVelocity(std::vector<double> velVec)
{
    assert(velVec.size() == linkVec.size() - 2); // 去掉头尾的虚杆
    for (decltype (linkVec.size()) i = 1; i < linkVec.size() - 1; i++)
    {
        linkVec[i].setJointVel(velVec[i - 1]);
    }
}

/* 计算机器人雅克比矩阵 */
Vector Robot::getJacobian() const
{
    std::vector<Matrix> matrixVec;

    std::vector<double> vec(4, 1);
    Matrix tmpMatrix(vec, 4);
    Matrix jacobian(6, linkVec.size() - 2);
    for (decltype (linkVec.size()) i = 1; i < linkVec.size(); i++)
    {
        matrixVec.push_back(linkVec[i].getTransMatrix().reverseMatrix());
    }
    for (int i = matrixVec.size() - 2; i >= 0; i--)
    {
        matrixVec[i] *= matrixVec[i + 1];
    }
    matrixVec[0].showMatrix();

    for (int i = 0; i < int(matrixVec.size()) - 1; i++)
    {
        Matrix diffMatrix = Robot::getDiffMatrix(matrixVec[i], linkVec[i + 1].getJointType());
        jacobian.insert(diffMatrix, 0, 6, i, i + 1);
    }

    return jacobian;
}


Vector Robot::getDiffMatrix(const Matrix &matrix, Joint_E jointType)
{
    Matrix rTranspose = matrix.getSubMatrix(0, 3, 0, 3).transpose();
    Vector position = matrix.getSubMatrix(0, 3, 3, 4);

    double px = position[0];
    double py = position[1];
    double pz = position[2];
    std::vector<double> vec{
         0, -pz,  py,
        pz,   0,  px,
       -py,  px,   0,
    };

    Matrix antiSymmetryMatrix(vec, 3, 3);

    antiSymmetryMatrix.showMatrix();


    Matrix tmpMatrix = rTranspose * antiSymmetryMatrix;

    tmpMatrix.showMatrix();


    tmpMatrix *= -1;
    Matrix diffMatrix(6, 6);
    diffMatrix.insert(rTranspose, 0, 3, 0, 3);
    diffMatrix.insert(rTranspose, 3, 6, 3, 6);
    diffMatrix.insert(tmpMatrix, 0, 3, 3, 6);

        diffMatrix.showMatrix();

    Vector ret(6, 1);
    if (jointType == JOINT_TRANSLATE)
    {
        ret = diffMatrix.getSubMatrix(0, 6, 2, 3);
    }
    else if (jointType == JOINT_REVOLUTE)
    {
        ret = diffMatrix.getSubMatrix(0, 6, 5, 6);
    }
    else
    {
        assert(false);
    }
    return ret;
}

std::vector<double> Robot::getJointVelocity() const
{
    std::vector<double> velVec;
    for (decltype (linkVec.size()) i = 1; i < linkVec.size() - 1; i++)
    {
        velVec.push_back(linkVec[i].getJointVel());
    }
    return velVec;
}

/*
 * posConVec：在某时刻，连杆位置应满足的约束条件
 * velConVec：在某时刻，连杆速度应满足的约束条件
 */
PlanVec Robot::motionPlan(ConVec &posConVec, ConVec &velConVec)
{

}


