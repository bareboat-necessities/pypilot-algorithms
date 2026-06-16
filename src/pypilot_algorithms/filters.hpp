#pragma once

#include "angles.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
class LowPass {
public:
    LowPass() : value_(Real(0)), initialized_(false) {}

    void reset() { value_ = Real(0); initialized_ = false; }
    void set(Real value) { value_ = value; initialized_ = true; }

    Real update(Real sample, Real alpha) {
        alpha = clamp(alpha, Real(0), Real(1));
        if (!initialized_) {
            value_ = sample;
            initialized_ = true;
            return value_;
        }
        value_ += alpha * (sample - value_);
        return value_;
    }

    Real value() const { return value_; }
    bool initialized() const { return initialized_; }

private:
    Real value_;
    bool initialized_;
};

template<typename Real = float>
class AngleLowPass {
public:
    AngleLowPass() : value_deg_(Real(0)), initialized_(false) {}

    void reset() { value_deg_ = Real(0); initialized_ = false; }
    void set(Real value_deg) { value_deg_ = wrap_180_deg(value_deg); initialized_ = true; }

    Real update(Real sample_deg, Real alpha) {
        alpha = clamp(alpha, Real(0), Real(1));
        sample_deg = wrap_180_deg(sample_deg);
        if (!initialized_) {
            value_deg_ = sample_deg;
            initialized_ = true;
            return value_deg_;
        }
        value_deg_ = wrap_180_deg(value_deg_ + alpha * wrap_180_deg(sample_deg - value_deg_));
        return value_deg_;
    }

    Real value() const { return value_deg_; }
    bool initialized() const { return initialized_; }

private:
    Real value_deg_;
    bool initialized_;
};

template<typename Real = float>
class Derivative {
public:
    Derivative() : last_(Real(0)), derivative_(Real(0)), initialized_(false) {}

    void reset() { last_ = Real(0); derivative_ = Real(0); initialized_ = false; }
    void set(Real value) { last_ = value; derivative_ = Real(0); initialized_ = true; }

    Real update(Real sample, Real dt_s) {
        if (!initialized_ || dt_s <= Real(0)) {
            set(sample);
            return Real(0);
        }
        derivative_ = (sample - last_) / dt_s;
        last_ = sample;
        return derivative_;
    }

    Real value() const { return derivative_; }

private:
    Real last_;
    Real derivative_;
    bool initialized_;
};

template<typename Real = float>
class AngleDerivative {
public:
    AngleDerivative() : last_deg_(Real(0)), derivative_deg_s_(Real(0)), initialized_(false) {}

    void reset() { last_deg_ = Real(0); derivative_deg_s_ = Real(0); initialized_ = false; }
    void set(Real value_deg) { last_deg_ = wrap_180_deg(value_deg); derivative_deg_s_ = Real(0); initialized_ = true; }

    Real update(Real sample_deg, Real dt_s) {
        sample_deg = wrap_180_deg(sample_deg);
        if (!initialized_ || dt_s <= Real(0)) {
            set(sample_deg);
            return Real(0);
        }
        derivative_deg_s_ = wrap_180_deg(sample_deg - last_deg_) / dt_s;
        last_deg_ = sample_deg;
        return derivative_deg_s_;
    }

    Real value() const { return derivative_deg_s_; }

private:
    Real last_deg_;
    Real derivative_deg_s_;
    bool initialized_;
};

} // namespace pypilot_algorithms
