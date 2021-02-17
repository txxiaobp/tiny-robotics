#include "motion_planning.h"
#include <cassert>

MotionPlanning::MotionPlanning(double timeIntervel,
                               double timeStep,
                               std::vector<std::pair<double,double>> &posConstraint,
                               std::vector<std::pair<double,double>> &velConstraint,
                               std::vector<std::pair<double,double>> &accelConstraint)
    : timeIntervel(timeIntervel)
    , timeStep(timeStep)
    , posConstraint(posConstraint)
    , velConstraint(velConstraint)
    , accelConstraint(accelConstraint)
{
    assert(timeIntervel > 0 && timeStep > 0);
    assert(timeStep <= timeIntervel);

    for (auto pair : posConstraint)
    {
        assert(pair.first >=0 && pair.first <= timeIntervel);
    }
    for (auto pair : velConstraint)
    {
        assert(pair.first >=0 && pair.first <= timeIntervel);
    }
}

MotionPlanning::~MotionPlanning()
{

}
