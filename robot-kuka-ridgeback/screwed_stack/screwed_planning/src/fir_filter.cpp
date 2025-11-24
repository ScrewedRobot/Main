

#include "fir_filter.h"


FIRFilter::FIRFilter() {
    // Initialize the filter state
    for (std::size_t i = 0; i < FIR_N; ++i) {
        _buf[i] = 0.0f;
    }
    _idx = 0;
}

double FIRFilter::process(double input) {
    // Update the buffer with the new input
    _buf[_idx] = input;

    // Compute the output using the FIR coeffs
    double output = 0.0f;
    std::size_t buf_idx = _idx;

    for (std::size_t k = 0; k < FIR_N; ++k) {
        output += fir_lp[k] * _buf[buf_idx];

        // Handle decrement and wrap-around
        if (buf_idx-- == 0) 
            buf_idx = FIR_N - 1;
    }

    // write index
    if (++_idx == FIR_N)
        _idx = 0;


    return output;
}

void FIRFilter::reset() {
    for (auto &v : _buf) {
        v = 0.0f;
    }
    _idx = 0;
}

SGFilter::SGFilter()
    : W_(0), P_(0), D_(0), half_(0), idx_(0), initialized_(false)
{}


SGFilter::SGFilter(int window_size, int poly_order, int derivative_order = 0)
    : W_(window_size), 
      P_(poly_order), 
      D_(derivative_order),
      half_(0),
      idx_(0),
      initialized_(false)
{
    if (W_ % 2 == 0 || P_ >= W_ || D_ < 0|| D_ > P_)
        throw std::invalid_argument("Invalid parameters for Savitzky-Golay filter");
    half_ = W_ / 2;
    buf_.resize(W_, 0.0);
    this->computeCoefficients();
}

double SGFilter::process(double input) {
    if (coeffs_.empty()) {
        throw std::runtime_error("SGFilter not initialized with valid parameters!");
    }

    if (!initialized_) {
        std::fill(buf_.begin(), buf_.end(), input);
        initialized_ = true;
        idx_ = 0;
    } else {
        buf_[idx_] = input;
        idx_ = (idx_ + 1) % W_;
    }

    double output = 0.0;
    for (int i = 0; i < W_; ++i) {
        int bi = (idx_ + i) % W_;
        output += coeffs_[i] * buf_[bi];
    }
    return output;
}

void SGFilter::computeCoefficients() {

    Eigen::MatrixXd A(W_, P_ + 1);
    for (int i = 0; i < W_; ++i) {
        double x = static_cast<double>(i - half_);
        double xp = 1.0;
        for (int j = 0; j <= P_; ++j) {
            A(i, j) = xp;
            xp *= x;
        }
    }

    Eigen::MatrixXd ATAinv = (A.transpose() * A).inverse();
    Eigen::MatrixXd pinv = ATAinv * A.transpose();

    double scale = 1.0;
    for (int i = 2; i <= D_; ++i) scale *= i;
    this->coeffs_.resize(W_);
    for (int i = 0; i < W_; ++i) {
        this->coeffs_[i] = pinv(D_, i) * scale;
    }
    buf_.resize(W_, 0.0);
}


