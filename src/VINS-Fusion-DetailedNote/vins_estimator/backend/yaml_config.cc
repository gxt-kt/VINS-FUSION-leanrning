#include "yaml_config.h"

#include <debugstream/debugstream.h>
#include <debugstream/fmt.h>

#include <filesystem>
#include <iostream>
#include <thread>

YAML::Node ReadConfigYamlFile(const std::string& yaml_config_path) {
  YAML::Node res;
  std::cout << __PRETTY_FUNCTION__ << ": " << std::endl;
  std::cout << "BEGIN READ FILE: " << yaml_config_path << std::endl;
  bool read_successful_flag = false;
  try {
    // Load the YAML file
    res = YAML::LoadFile(yaml_config_path);
    read_successful_flag = true;
  } catch (const YAML::Exception& e) {
    std::cerr << "Error while reading the YAML file: " << yaml_config_path
              << e.what() << std::endl;
  }
  if (!read_successful_flag) {
    std::cerr << "Error while reading the YAML file!" << yaml_config_path
              << std::endl;
    std::cerr << "Error while reading the YAML file!" << yaml_config_path
              << std::endl;
    std::cerr << "Error while reading the YAML file!" << yaml_config_path
              << std::endl;
    std::terminate();
  }
  std::cout << "Read yaml config file successfully! " << yaml_config_path
            << std::endl;
  return res;
}

YAML::Node yaml_config = ReadConfigYamlFile([]() {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path yaml_path = current_dir / "yaml_config.yaml";
  std::cout << "yaml file path: " << yaml_path << std::endl;
  return yaml_path;
}());

YAML::Node yaml_quantize;

std::thread save_yaml_quantize([]() {
  auto save_file_path = []() {
    std::filesystem::path current_dir =
        std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path yaml_path = current_dir / "yaml_quantize.yaml";
    std::cout << "yaml file path: " << yaml_path << std::endl;
    return yaml_path;
  }();
  gDebugWarn() << gxt::format("save yaml quantize file to <{}>",
                              save_file_path.c_str());

  while (true) {
    gxt::Sleep(3);
    std::ofstream fout(save_file_path);
    fout << yaml_quantize;
    fout.close();
  }
});
// YAML::Node yaml_quantize = ReadConfigYamlFile();
