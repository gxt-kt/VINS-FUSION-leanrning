# add_library(ls_solve vertex.cc edge.cc problem.cc fpm.cpp)
add_library(ls_solve vertex.cc edge.cc problem.cc yaml_config.cc)
# gxt: use yaml to config value
find_package(yaml-cpp REQUIRED)  
find_package(debugstream REQUIRED)  
target_link_libraries(ls_solve  yaml-cpp debugstream) 
