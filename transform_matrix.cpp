#include "transform_matrix.h"
#include <vector>
#include <cassert>
#include <cmath>


Matrix TransformMatrix::rotate(Axis_E axis, double angle)
{
    std::vector<double> valueVec;
    switch (axis)
    {
    case AXIS_X:
        valueVec = std::vector<double>{
             1,           0,           0,   0,
             0,  cos(angle), -sin(angle),   0,
             0,  sin(angle),  cos(angle),   0,
             0,           0,           0,   1
        };
        break;
    case AXIS_Y:
        valueVec = std::vector<double>{
             cos(angle),   0,  sin(angle), 0,
                      0,   1,           0, 0,
            -sin(angle),   0,  cos(angle), 0,
                      0,   0,           0, 1
        };
        break;
    case AXIS_Z:
        valueVec = std::vector<double>{
             cos(angle), -sin(angle), 0, 0,
             sin(angle),  cos(angle), 0, 0,
                      0,           0, 1, 0,
                      0,           0, 0, 1
        };
        break;
    default:
        assert(false);
    }

    return Matrix(valueVec, 4, 4);
}

Matrix TransformMatrix::translate(Axis_E axis, double distance)
{
    std::vector<double> valueVec;
    switch (axis)
    {
    case AXIS_X:
        valueVec = std::vector<double>{
            1,   0,   0,   distance,
            0,   1,   0,          0,
            0,   0,   1,          0,
            0,   0,   0,          1,
        };
        break;
    case AXIS_Y:
        valueVec = std::vector<double>{
            1,   0,   0,          0,
            0,   1,   0,   distance,
            0,   0,   1,          0,
            0,   0,   0,          1,
        };
        break;
    case AXIS_Z:
        valueVec = std::vector<double>{
            1,   0,   0,          0,
            0,   1,   0,          0,
            0,   0,   1,   distance,
            0,   0,   0,          1,
        };
        break;
    default:
        assert(false);
    }

    return Matrix(valueVec, 4, 4);
}
