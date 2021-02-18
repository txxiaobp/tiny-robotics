#include "scara.h"
#include <cassert>
#include <cmath>

Scara::Scara()
{

}

/*
 * 常见的SCARA机器人也满足Pieper准则
 */
std::vector<double> Scara::inverseKinematics(Matrix &endPos)
{
    std::vector<double> res;
    double l1 = linkVec[2].getXBias();
    double l2 = linkVec[3].getXBias();
    double d1 = linkVec[1].getZBias();

    assert(endPos.getRow() == endPos.getCol() && endPos.getRow() == 4);
    Vector pVec = endPos.getSubMatrix(0, 3, 3, 4);
    double px = pVec[0];
    double py = pVec[1];
    double pz = pVec[2];

    Vector nVec = endPos.getSubMatrix(0, 3, 0, 1);
    double nx = nVec[0];
    double ny = nVec[1];

    double a = (px * px + py * py + l1 * l1 - l2 * l2) / 2 / l1 / sqrt(px * px + py * py);
    double theta1 = atan(a / sqrt(1 - a * a)) - atan(px / py);

    double r = sqrt(px * px + py * py);
    double phi = atan(px / py);
    double theta2 = atan(r * cos(theta1 + phi) / (r * sin(theta1 + phi) - l1));

    double d3 = d1 - pz;

    double theta4 = atan( (ny * cos(theta1) - nx * sin(theta1)) / (ny * sin(theta1) + nx * cos(theta1)) ) - theta2;

    res.push_back(theta1);
    res.push_back(theta2);
    res.push_back(d3);
    res.push_back(theta4);

    return res;
}
