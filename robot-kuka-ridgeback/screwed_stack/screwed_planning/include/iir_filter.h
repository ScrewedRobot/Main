#pragma once

#include <cmath>

class Butterworth1st {
public:
    Butterworth1st(double fs, double fc);
    double process(double input);

private:
    double b0, b1, a1, z1, x1, y1;
};

class Butterworth2nd {
public:
    Butterworth2nd(double fs, double fc);
    double process(double input);

private:
    double b0, b1, b2, a1, a2, z1, z2;
};