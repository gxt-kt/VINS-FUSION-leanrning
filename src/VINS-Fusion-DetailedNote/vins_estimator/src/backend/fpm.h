#pragma once

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <type_traits>

// #include "problem.h"

#include "common.hpp"
#include "vertex.h"

// template <typename Dtype>
// void SetValue(int bit_width, int fl) {
//   Dtype max_data = (std::pow(2, bit_width - 1) - 1) * std::pow(2, -fl);
//   Dtype min_data = -std::pow(2, bit_width - 1) * std::pow(2, -fl);
// }

// template <typename Derived>
// Derived GetAbsMax(const Eigen::MatrixBase<Derived>& matrix) {
//   return matrix.cwiseAbs().maxCoeff();
// }

// template <typename Dtype, typename Derived>
// void CutOffMatrix(Eigen::MatrixBase<Derived>& matrix, int bit_width = 32,
//                   int fl = 16) {
//   Dtype max_data = (std::pow(2, bit_width - 1) - 1) * std::pow(2, -fl);
//   Dtype min_data = -std::pow(2, bit_width - 1) * std::pow(2, -fl);
//   gDebugWarn(max_data);
//   gDebugWarn(min_data);
//   for (int i = 0; i < matrix.rows; i++) {
//     for (int j = 0; j < matrix.cols; j++) {
//       matrix(i, j) = std::max(std::min(matrix(i, j), max_data), min_data);

//       // Round data
//       matrix(i, j) /= std::pow(2, -fl);
//       // case QuantizationParameter_Rounding_NEAREST
//       matrix(i, j) = std::round(matrix(i, j));
//       matrix(i, j) *= std::pow(2, -fl);
//     }
//   }
// }

// template <typename Derived, typename F>
// void MatrixSetRandomValue(Eigen::MatrixBase<Derived>& matrix, F f,
//                           typename Derived::Scalar min = -10,
//                           typename Derived::Scalar max = 10) {
//   for (int i = 0; i < matrix.rows(); i++) {
//     for (int j = 0; j < matrix.cols(); j++) {
//       matrix(i, j) = f(min, max);
//     }
//   }
// }

// struct MatrixQuantify {
//   using MatrixDynamic = Eigen::Matrix<double, Eigen::Dynamic,
//   Eigen::Dynamic>; int bit_width_ = 32; int il_ = 16;
//   // int fl=16;
//   double current_max_value_ = -1;
//   double max_value_ = -1;
//   // MatrixQuantify(){}

//   MatrixQuantify(double* matrix_data,int rows,int cols)
//       : matrix_(matrix_data, rows, cols) {
//     max_value_ = MatrixGetAbsMax();
//     current_max_value_ = max_value_;
//     il_ = (int)std::ceil(std::log2(max_value_));
//     il_=il_<0?0:il_;
//     gDebug(max_value_);
//     gDebug(il_);
//   };

//   void Update() {
//     current_max_value_=MatrixGetAbsMax();
//     max_value_ = std::max(current_max_value_,max_value_);
//     il_ = (int)std::ceil(std::log2(max_value_));
//   }

//   double MatrixGetAbsMax() { return matrix_.cwiseAbs().maxCoeff(); }
//   // MatrixDynamic& matrix_;
//   Eigen::Map<Eigen::MatrixXd> matrix_;
// };

// extern std::map<unsigned long, MatrixQuantify> verticies_quantify;

// inline void QuantifyInitAddVertex(double* matrix_data,int rows,int cols) {
// }
// inline void QuantifyFirstUpdate() {
//   // 以前没添加过
//   // if (verticies_quantify.find((unsigned long)matrix_data) ==
//   //     verticies_quantify.end()) {
//   //   verticies_quantify.insert(
//   //       {(unsigned long)matrix_data, MatrixQuantify(matrix_data,
//   rows,cols)});
//   // }
//   for(auto& verticies_quantify_i:verticies_quantify) {
//     verticies_quantify_i.second.Update();

//   }
// }

// inline void QuantizeAddPoseVertex(std::shared_ptr<Vertex> vertex) {

// }

template <typename Derived>
typename Derived::Scalar VectorGetAbsMax(
    const Eigen::MatrixBase<Derived>& matrix, int begin = -1, int end = -1) {
  using value_type = typename Derived::Scalar;
  if (begin == -1) return matrix.cwiseAbs().maxCoeff();
  if (begin != -1 && end == -1) {
    value_type max_ = -1;
    for (int i = begin; i < matrix.size(); i++) {
      max_ = std::max(std::abs(matrix[i]), max_);
    }
    return max_;
  }
  if (begin != -1 && end != -1) {
    value_type max_ = -1;
    for (int i = begin; i < end; i++) {
      max_ = std::max(std::abs(matrix[i]), max_);
    }
    return max_;
  }
  // handle the case when both begin and end are -1 or other invalid input
  // return a default value or throw an error, depending on the desired behavior
}

template <typename Derived>
typename Derived::Scalar MatrixGetAbsMax(
    const Eigen::MatrixBase<Derived>& matrix) {
  return matrix.cwiseAbs().maxCoeff();
}

template <typename Derived>
void QuantizeParamCutOff(Eigen::MatrixBase<Derived>& matrix, int bit_width = 32,
                         int fl = 16, int begin = -1, int end = -1) {
  using value_type = typename Derived::Scalar;
  double max_data = (std::pow(2, bit_width - 1) - 1) * std::pow(2, -fl);
  double min_data = -std::pow(2, bit_width - 1) * std::pow(2, -fl);
  // gDebugWarn(max_data);
  // gDebugWarn(min_data);

  auto CutOff=[&](value_type& value){
      value = std::max(std::min(value, max_data), min_data);
      // Round data
      value /= std::pow(2, -fl);
      // case QuantizationParameter_Rounding_NEAREST
      value = std::round(value);
      value *= std::pow(2, -fl);
  };

  if (begin == -1) {
    for (int i = 0; i < matrix.size(); i++) {
      CutOff(matrix(i));
    }
  }
  if (begin != -1 && end == -1) {
    for (int i = begin; i < matrix.size(); i++) {
      CutOff(matrix(i));
    }
  }
  if (begin != -1 && end != -1) {
    for (int i = begin; i < end; i++) {
      CutOff(matrix(i));
    }
  }
  
  // for (int i = 0; i < matrix.rows(); i++) {
  //   for (int j = 0; j < matrix.cols(); j++) {
  //     matrix(i, j) = std::max(std::min(matrix(i, j), max_data), min_data);
  //     // Round data
  //     matrix(i, j) /= std::pow(2, -fl);
  //     // case QuantizationParameter_Rounding_NEAREST
  //     matrix(i, j) = std::round(matrix(i, j));
  //     matrix(i, j) *= std::pow(2, -fl);
  //   }
  // }
}

extern double pose_quaternion_max;
extern int quantize_pose_quaternion_bit_width;
extern int quantize_pose_quaternion_il;

extern double pose_transition_max;
extern int quantize_pose_transition_bit_width;
extern int quantize_pose_transition_il;

extern double imu_speed_max;
extern int quantize_imu_speed_bit_width;
extern int quantize_imu_speed_il;

extern double imu_accbias_max;
extern int quantize_imu_accbias_bit_width;
extern int quantize_imu_accbias_il;

extern double imu_gyrobias_max;
extern int quantize_imu_gyrobias_bit_width;
extern int quantize_imu_gyrobias_il;

extern double inverse_depth_max;
extern int quantize_inverse_depth_bit_width;
extern int quantize_inverse_depth_il;

// Hessian
extern double hessian_max;
extern int quantize_hessian_bit_width;
extern int quantize_hessian_il;

// b
extern double b_max;
extern int quantize_b_bit_width;
extern int quantize_b_il;

// flag
extern bool use_gxt_backend;
extern bool enable_quantize;
