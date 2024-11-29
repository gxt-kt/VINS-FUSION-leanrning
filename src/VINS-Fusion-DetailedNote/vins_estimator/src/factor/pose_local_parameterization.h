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

    virtual void Plus(const VecX &delta) override;
    std::string TypeInfo() const override { return "VertexPose"; }
    // virtual bool Plus(const VecX &delta) const;
    // virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    // virtual int GlobalSize() const { return 7; };
    // virtual int LocalSize() const { return 6; };
};
