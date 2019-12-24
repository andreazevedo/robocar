#include "inference/util.h"

#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>

namespace robocar {
namespace inference {

std::vector<std::string> loadLabels(const std::string& labelsFilePath) {
  std::ifstream in(labelsFilePath);
  if (!in) {
    throw std::runtime_error("Cannot open file: " + labelsFilePath);
  }

  std::vector<std::string> classes;

  std::string str;
  while (std::getline(in, str)) {
    classes.push_back(str);
  }
  in.close();

  return classes;
}

}  // namespace inference
}  // namespace robocar

