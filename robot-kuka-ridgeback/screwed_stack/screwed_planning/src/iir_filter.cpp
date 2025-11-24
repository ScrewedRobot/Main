

#include "iir_filter.h"

// Butterworth1st implementation
Butterworth1st::Butterworth1st(double fs, double fc) {
    double K = std::tan(M_PI * fc / fs); // pre-warp Constant
    b0 = K / (1.0 + K);  
    b1 = b0;
    a1 = (1.0 - K) / (1.0 + K);
    z1 = 0.0; // initialize state
}

double Butterworth1st::process(double input) {
    double output = b0 * input + z1;    // compute output
    z1 = b1 * input - a1 * output;      // update state
    return output;
}

// Butterworth2nd implementation
Butterworth2nd::Butterworth2nd(double fs, double fc) {
    double K = std::tan(M_PI * fc / fs); // pre-warp Constant
    double norm = 1 + sqrt(2)*K + K*K;   // normalization factor
    b0 = K*K / norm;
    b1 = 2*b0;
    b2 = b0;
    a1 = 2 * (K*K - 1) / norm;
    a2 = (1 - sqrt(2)*K + K*K) / norm;
    z1 = 0.0; // initialize state
    z2 = 0.0; // initialize state
}

double Butterworth2nd::process(double input) {
    double output = b0 * input + z1;    // compute output
    z1 = b1 * input + z2 - a1*output;   // update state
    z2 = b2 * input - a2 * output;      // update state
    return output;
}