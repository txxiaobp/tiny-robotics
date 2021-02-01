#ifndef TRANSFORMMATRIX_H
#define TRANSFORMMATRIX_H

#include "matrix.h"

typedef enum
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z
}Axis_E;


class TransformMatrix
{
public:
    static Matrix rotate(Axis_E axis, double angle);
    static Matrix translate(Axis_E axis, double distance);
};

#endif // TRANSFORMMATRIX_H
