#ifndef SCARA_H
#define SCARA_H

#include "robot.h"

class Scara : public Robot
{
public:
    Scara();
    ~Scara() {}
    std::vector<double> inverseKinematics(Matrix &endPos);
};

#endif // SCARA_H
