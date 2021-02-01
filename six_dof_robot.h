#ifndef SIXFODROBOT_H
#define SIXFODROBOT_H

#include "robot.h"

class SixDOFRobot : public Robot
{
public:
    SixDOFRobot();
    ~SixDOFRobot() {}
    std::vector<double> inverseKinematics(Vector &endPos);
};

#endif // SIXFODROBOT_H
