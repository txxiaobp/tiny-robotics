#include "six_fod_robot.h"

SixFODRobot::SixFODRobot()
    : Robot()
{

}

/*
 * 常见的6自由度工业机器人满足Pieper准则，
 * 即腕点为后三个坐标系的公共原点
 */
std::vector<double> SixFODRobot::inverseKinematics(Vector &endPos)
{

}
