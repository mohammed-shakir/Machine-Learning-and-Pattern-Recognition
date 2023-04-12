#pragma once

struct Coordinate {
    float x;
    float y;
    float z;

    float x_angle;
    float y_angle;
    float z_angle;

    enum Type {
        STD,
        MEAN
    } type;
};