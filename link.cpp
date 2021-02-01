#include "link.h"
#include <cassert>

Link::Link(Joint_E jointType, double initXBias, double initXAngle, double initZBias, double initZAngle)
    : jointType(jointType)
    , xBias(initXBias)
    , xAngle(initXAngle)
    , zBias(initZBias)
    , zAngle(initZAngle)
    , fixed(false)
{

}


Link::Link()
    : jointType(JOINT_NONE)
    , xBias(0)
    , xAngle(0)
    , zBias(0)
    , zAngle(0)
    , fixed(false)
{

}

void Link::setFix(bool fixed)
{
    this->fixed = fixed;
}

bool Link::isFixed() const
{
    return fixed;
}

double Link::getXBias() const
{
    return xBias;
}

double Link::getXAngle() const
{
    return xAngle;
}

double Link::getZBias() const
{
    return zBias;
}

double Link::getZAngle() const
{
    return zAngle;
}

void Link::move(double increment)
{
    assert(!isFixed());

    switch (jointType)
    {
    case JOINT_REVOLUTE:
        xAngle += increment; // 如果该连杆相对于前一个连杆转动，则关节运动会改变 xAngle 的数值
        break;
    case JOINT_TRANSLATE:
        xBias += increment; // 如果该连杆相对于前一个连杆移动，则关节运动会改变 xBias 的数值
        break;
    default:
        assert(false);
    }
}
