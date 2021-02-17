#include "linear_with_parabola_transition_planning.h"
#include <cassert>
#include <cmath>
#include <iostream>

LinearWithParabolaTransitionPlanning::LinearWithParabolaTransitionPlanning(double timeIntervel,
                                                                           double timeStep,
                                                                           std::vector<std::pair<double,double>> &posConstraint,
                                                                           std::vector<std::pair<double,double>> &velConstraint,
                                                                           std::vector<std::pair<double,double>> &accelConstraint,
                                                                           double maxAcceleration)
    : MotionPlanning(timeIntervel, timeStep, posConstraint, velConstraint, accelConstraint)
    , maxAcceleration(maxAcceleration)
{
    assert(posConstraint.size() >= 2);
    assert(velConstraint.empty());
    assert(accelConstraint.empty());

    for (int i = 0; i < static_cast<int>(posConstraint.size()); i++)
    {
        assert(posConstraint[i].first >= 0 && posConstraint[i].first <= timeIntervel);
        if (i == static_cast<int>(posConstraint.size()) - 1)
        {
            continue;
        }
        assert(posConstraint[i].first < posConstraint[i + 1].first);
    }
}


bool LinearWithParabolaTransitionPlanning::plan(std::vector<std::pair<double,double>> &posVec,
                                   std::vector<std::pair<double,double>> &velVec,
                                   std::vector<std::pair<double,double>> &accelVec)
{
    std::vector<double> tmptimeVec;
    std::vector<double> tmpPosVec;
    std::vector<double> tmpVelVec;
    std::vector<std::pair<double,double>> tmpAccelVec;

    tmptimeVec.clear();
    tmpPosVec.clear();
    for (auto pair : posConstraint)
    {
        tmptimeVec.push_back(pair.first);
        tmpPosVec.push_back(pair.second);
    }

    calDurationVelocityAndAcceleration(tmptimeVec, tmpPosVec, tmpVelVec, tmpAccelVec);

    int timeCount = ceil(timeIntervel / timeStep) + 1;
    double time = 0;
    double floorPosIndex = 0;

    double timeIndex = 0;
    for (int i = 0; i < timeCount; i++, time += timeStep)
    {
        double accel = tmpAccelVec[timeIndex].first;
        double accelTime = tmpAccelVec[timeIndex].second;
        double floorTime = tmptimeVec[timeIndex];
        double timeDuration = tmptimeVec[timeIndex + 1] - tmptimeVec[timeIndex];

        while (time - floorTime > timeDuration)
        {
            timeIndex++;
            accel = tmpAccelVec[timeIndex].first;
            accelTime = tmpAccelVec[timeIndex].second;
            floorTime = tmptimeVec[timeIndex];
            timeDuration = tmptimeVec[timeIndex + 1] - tmptimeVec[timeIndex];
        }

        double pos = 0;
        double nextFloorTime = tmptimeVec[timeIndex + 1];
        double nextAccelTime = tmpAccelVec[timeIndex + 1].second;
        if (time - floorTime <= accelTime / 2)
        {
            pos = floorPos + 0.5 * accel * pow(time - floorTime, 2);
        }
        else if (time <= nextFloorTime - nextAccelTime / 2)
        {
            pos = floorPos + 0.5 * accel * pow(accelTime, 2) + accel * accelTime * (time - floorTime);
        }
        else
        {
            assert(time > nextFloorTime - nextAccelTime / 2);
            double nextAccel = tmpAccelVec[timeIndex + 1].first;
            double tmpTime = timeDuration - nextAccelTime / 2 - accelTime / 2;
            pos = floorPos + 0.5 * accel * pow(accelTime, 2) + accel * accelTime * tmpTime + 0.5 * nextAccel * pow(time - nextFloorTime + nextAccelTime / 2, 2);
        }
        posVec.push_back(std::make_pair(time, pos));
    }

    return true;
}

void LinearWithParabolaTransitionPlanning::calDurationVelocityAndAcceleration(std::vector<double> &timeVec,
                                                                              std::vector<double> &posVec,
                                                                              std::vector<double> &velVec,
                                                                              std::vector<std::pair<double,double>> &accelVec)
{
    int size = static_cast<int>(posVec.size());
    double timeDuration12 = timeVec[1] - timeVec[0];
    double displacement12 = posVec[1] - posVec[0];
    double acceleration12 = calAcceleration(0, posVec[1] - posVec[0]);

    double timeStartTransition = timeDuration12 - sqrt(pow(timeDuration12, 2) - 2 * displacement12 / acceleration12);
    double velStart = displacement12 / (timeDuration12 - timeStartTransition / 2);

    velVec = std::vector<double>(posVec.size() - 1, 0);
    accelVec = std::vector<std::pair<double,double>>(posVec.size());
    assert(timeVec.size() == posVec.size());

    velVec[0] = velStart;
    accelVec[0] = std::make_pair(acceleration12, timeStartTransition);

    double timeDurationEnd = timeVec[size - 1] - timeVec[size - 2];
    double displacementEnd = posVec[size - 1] - posVec[size - 2];

    double accelerationEnd = calAcceleration(posVec[size - 1] - posVec[size - 2], 0);
    double timeEndTransition = timeDurationEnd - sqrt(pow(timeDurationEnd, 2) + 2 * displacementEnd / accelerationEnd);
    double velEnd = displacementEnd / (timeDurationEnd - timeEndTransition / 2);
    velVec[posVec.size() - 2] = velEnd;
    accelVec[posVec.size() - 1] = std::make_pair(accelerationEnd, timeEndTransition);

    for (int i = 1; i < size - 1; i++)
    {
        if (i != size - 2)
        {
            double timeDuration = timeVec[i + 1] - timeVec[i];
            double displacement = posVec[i + 1] - posVec[i];
            double velocity = displacement / timeDuration;
            velVec[i] = velocity;
        }
        double accel = calAcceleration(velVec[i - 1], velVec[i]);
        double accelTime = (velVec[i] - velVec[i - 1]) / accel;
        accelVec[i] = std::make_pair(accel, accelTime);
    }
}

double LinearWithParabolaTransitionPlanning::calAcceleration(double velStart, double velEnd)
{
    if (velEnd >= velStart)
    {
        return maxAcceleration;
    }
    else
    {
        return -maxAcceleration;
    }
}
