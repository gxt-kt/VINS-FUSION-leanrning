#pragma once

#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "fpm.h"

#include "common.hpp"

// #define ADDCONFIG(a,b) decltype(tebconfig.a.b) b=a[#b].as<decltype(tebconfig.a.b)>();\
//   tebconfig.a.b=b; \
//   gDebugCol3(tebconfig.a.b);

#define ADDCONFIG(a,b) b=a[#b].as<decltype(b)>();\
    gDebugCol3(b);

static std::string yaml_config_path = []() -> std::string {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path yaml_path = current_dir / "yaml_config.yaml";
  std::cout << "yaml file path: " << yaml_path << std::endl;
  return yaml_path;
}();

// static std::string my_yaml_config_path =
//     "/media/Projects/vins_fusion/src/VINS-Fusion-DetailedNote/"
//     "vins_estimator/src/backend/config.yaml";

inline bool GxtReadConfigXmlFile() {
  bool read_successful_flag = false;
  try {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(yaml_config_path);

    read_successful_flag = true;
    gDebugCol1("READ CONFIG FILE SUCCESSFULLY!");
    
    auto quantize = config["quantize"];
    ADDCONFIG(quantize,quantize_pose_quaternion_bit_width);
    ADDCONFIG(quantize,quantize_pose_quaternion_il);
    ADDCONFIG(quantize,quantize_pose_quaternion_bit_width);
    ADDCONFIG(quantize,quantize_pose_quaternion_il);

    ADDCONFIG(quantize,quantize_imu_speed_bit_width);
    ADDCONFIG(quantize,quantize_imu_speed_il);
    ADDCONFIG(quantize,quantize_imu_accbias_bit_width);
    ADDCONFIG(quantize,quantize_imu_accbias_il);
    ADDCONFIG(quantize,quantize_imu_gyrobias_bit_width);
    ADDCONFIG(quantize,quantize_imu_gyrobias_il);

    ADDCONFIG(quantize,quantize_inverse_depth_bit_width);
    ADDCONFIG(quantize,quantize_inverse_depth_il);

    ADDCONFIG(quantize,quantize_hessian_bit_width);
    ADDCONFIG(quantize,quantize_hessian_il);
    ADDCONFIG(quantize,quantize_b_bit_width);
    ADDCONFIG(quantize,quantize_b_il);

    auto flag = config["flag"];
    ADDCONFIG(flag,use_gxt_backend);
    ADDCONFIG(flag,enable_quantize);
  } catch (const YAML::Exception& e) {
    // std::cerr << "Error while reading the YAML file: " << e.what() <<
    // std::endl;
    gDebugError("[GXT] : Error while reading the YAML file:") << e.what();
  }

  if (read_successful_flag == false) {
    gDebugCol3("\n\n\n==========================================");
    gDebugCol3("[GXT] : Error while reading the YAML file!");
    gDebugCol3("[GXT] : Error while reading the YAML file!");
    gDebugError("[GXT] : Error while reading the YAML file!");
  }

  return 0;
}
