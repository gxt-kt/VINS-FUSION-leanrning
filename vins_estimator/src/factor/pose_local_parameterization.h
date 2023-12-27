/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"



class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

#include "problem.h"

class GxtPoseLocalParameterization : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    GxtPoseLocalParameterization() : Vertex(7, 6) {}

    virtual void Plus(const VecX &delta) override {
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
    std::string TypeInfo() const override { return "VertexPose"; }
    // virtual bool Plus(const VecX &delta) const;
    // virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    // virtual int GlobalSize() const { return 7; };
    // virtual int LocalSize() const { return 6; };
};
