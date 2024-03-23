#include "fpm.h"

// add fpm
// std::map<unsigned long, MatrixQuantify> verticies_quantify;

// MatrixQuantify quantize_pose;

// pose顶点 四元数 4
double pose_quaternion_max=-1;
int quantize_pose_quaternion_bit_width=16;
int quantize_pose_quaternion_il=1;

// pose顶点 平移 3
double pose_transition_max=-1;
int quantize_pose_transition_bit_width=16;
int quantize_pose_transition_il=4;

// imu的speed偏置 3
double imu_speed_max=-1;
int quantize_imu_speed_bit_width=16;
int quantize_imu_speed_il=2;

// imu的acc bias偏置 3
double imu_accbias_max=-1;
int quantize_imu_accbias_bit_width=16;
int quantize_imu_accbias_il=2;

// imu的gyro bias偏置 3
double imu_gyrobias_max=-1;
int quantize_imu_gyrobias_bit_width=16;
int quantize_imu_gyrobias_il=2;

// InverseDepth 1
double inverse_depth_max=-1;
int quantize_inverse_depth_bit_width=32;
int quantize_inverse_depth_il=7;


// Hessian
double hessian_max=-1;
int quantize_hessian_bit_width=64;
int quantize_hessian_il=32;

// b
double b_max=-1;
int quantize_b_bit_width=64;
int quantize_b_il=32;
