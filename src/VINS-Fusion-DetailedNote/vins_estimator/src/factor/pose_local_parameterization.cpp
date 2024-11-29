/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

void GxtPoseLocalParameterization::Plus(const VecX &delta) {
  VecX &parameters = Parameters();
  // Eigen::Map<const Eigen::Vector3d> _p(&parameters(0));
  Eigen::Map<const Eigen::Quaterniond> _q(&parameters(0) + 3);

  // Eigen::Map<const Eigen::Vector3d> dp(&delta(0));

  Eigen::Quaterniond dq =
      Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(&delta(0) + 3));

  // Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Quaterniond q_plus_delta = (_q * dq).normalized();

  // p = _p + dp;
  // q = (_q * dq).normalized();
  parameters.head<3>() += delta.head<3>();
  parameters[3] = q_plus_delta.x();
  parameters[4] = q_plus_delta.y();
  parameters[5] = q_plus_delta.z();
  parameters[6] = q_plus_delta.w();

  // return true;
}
