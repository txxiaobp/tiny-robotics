#ifndef SIXFODROBOT_H
#define SIXFODROBOT_H

#include "robot.h"

class SixFODRobot : public Robot
{
public:
    SixFODRobot();
    ~SixFODRobot() {}
    std::vector<double> inverseKinematics(Vector &endPos);
};

#endif // SIXFODROBOT_H
