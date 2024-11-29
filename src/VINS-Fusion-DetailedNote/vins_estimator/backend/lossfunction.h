#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>

namespace gxt {
class LossFunction {
public:
  LossFunction() {}
  virtual void Evaluate(double s, double rho[3]) const = 0;
};
class CauchyLoss : public LossFunction {
public:
  explicit CauchyLoss(double a = 1.0) : b_(a * a), c_(1 / b_) {}
  void Evaluate(double s, double rho[3]) const override {
    const double sum = 1.0 + s * c_;
    const double inv = 1.0 / sum;
    // 'sum' and 'inv' are always positive, assuming that 's' is.
    rho[0] = b_ * log(sum);
    rho[1] = std::max(std::numeric_limits<double>::min(), inv);
    rho[2] = -c_ * (inv * inv);
  }

private:
  // b = a^2.
  const double b_;
  // c = 1 / a^2.
  const double c_;
};

class HuberLoss : public LossFunction {
public:
  explicit HuberLoss(double a = 1.0) : a_(a), b_(a * a) {}
  void Evaluate(double s, double rho[3]) const override {
    if (s > b_) {
      // Outlier region.
      // 'r' is always positive.
      const double r = sqrt(s);
      rho[0] = 2.0 * a_ * r - b_;
      rho[1] = std::max(std::numeric_limits<double>::min(), a_ / r);
      rho[2] = -rho[1] / (2.0 * s);
    } else {
      // Inlier region.
      rho[0] = s;
      rho[1] = 1.0;
      rho[2] = 0.0;
    }
  }

private:
  const double a_;
  // c = 1 / a^2.
  const double b_;
};
  

}
