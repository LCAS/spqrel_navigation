#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <map>

#include <Eigen/Core>

typedef std::map<std::string, std::string> StringMap;
  
class SimpleYAMLParser {
 public:
  void load(const std::string filename);
  inline std::string getValue(const std::string key) {return _sm[key];}
  inline float getValueAsFloat(const std::string key) {return std::stof(_sm[key]);}
  inline int getValueAsInt(const std::string key) {return std::stoi(_sm[key]);}
  Eigen::Vector3f getValueAsVector3f(const std::string key);
  
 protected:
  StringMap _sm;
};
