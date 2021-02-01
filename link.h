#ifndef LINK_H
#define LINK_H

#include "matrix.h"

typedef enum
{
    JOINT_NONE,
    JOINT_REVOLUTE,
    JOINT_TRANSLATE,
    JOINT_MAX
}Joint_E;

class Link
{
public:
    Link(Joint_E jointType, double initXBias, double initXAngle, double initZBias, double initZAngle);
    Link();

    void setFix(bool fixed);
    bool isFixed() const;
    double getXBias() const;
    double getXAngle() const;
    double getZBias() const;
    double getZAngle() const;
    void move(double increment);

private:
    Joint_E jointType; //相对于前一个连杆的运动型式
    double xBias;      //Z轴相对于前一个连杆X轴的偏置
    double xAngle;     //Z轴相对于前一个连杆X轴的夹角
    double zBias;      //Z轴相对于后一个连杆Z轴的偏置
    double zAngle;     //Z轴相对于后一个连杆Z轴的夹角
    bool fixed;

    Matrix inertialMatrix;
    Vector posVec; //连杆位置
    Vector velVec; //连杆速度
};

#endif // LINK_H
