
#pragma once

#include <cstddef>
#include <array>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include "fir_coeffs.hpp"

class FIRFilter {
public:
    FIRFilter();
    double process(double input);
    void reset();

private:
    float _buf[FIR_N];
    std::size_t _idx;
};

class SGFilter {
public:
    SGFilter();
    SGFilter(int window_size, int poly_order, int derivative_order);
    double process(double input);

private:
    int W_, P_, D_, half_, idx_;
    bool initialized_;
    std::vector<double> coeffs_, buf_;

    void computeCoefficients();

};