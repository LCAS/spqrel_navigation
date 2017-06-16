#include "simple_yaml_parser.h"

void SimpleYAMLParser::load(const std::string filename){
  std::ifstream ifs(filename);
  if (!ifs.good()){
    std::cerr << "Error reading file " << filename << std::endl;
  }
  while (ifs.good()){
    std::string key, value;
    std::getline(ifs,key,':');
    key.erase(remove(key.begin(), key.end(), ' '), key.end()); //removes spaces
    std::getline(ifs,value);
    value.erase(remove(value.begin(), value.end(), ' '), value.end()); //removes spaces
    
    if (key.size())
      _sm[key] = value;
  }
}

Eigen::Vector3f SimpleYAMLParser::getValueAsVector3f(const std::string key){
  std::string string_value = getValue(key);
  string_value.erase(remove(string_value.begin(), string_value.end(), '['), string_value.end());
  string_value.erase(remove(string_value.begin(), string_value.end(), ']'), string_value.end());

  Eigen::Vector3f v(0.0,0.0,0.0);
  std::stringstream ss(string_value);
  int i = 0;
  while (ss.good()){
    std::string value;
    std::getline(ss,value, ',');
    if (i < 3)
      v[i] = std::stof(value);
    else
      std::cerr << "YAML Parser: More than 3 values provided. Ignoring exceeding value." << std::endl;
    i++;
  }
  if (i < 3)
    std::cerr << "YAML Parser: Less than 3 values provided. Filling remaining vector with 0." << std::endl;

  return v;
}
