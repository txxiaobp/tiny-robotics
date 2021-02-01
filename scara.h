#ifndef SCARA_H
#define SCARA_H

#include "robot.h"

class Scara : public Robot
{
public:
    Scara();
    ~Scara() {}
    std::vector<double> inverseKinematics(Vector &endPos);
};

#endif // SCARA_H
