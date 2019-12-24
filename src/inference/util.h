#pragma once

#include <string>
#include <vector>

namespace robocar {
namespace inference {

std::vector<std::string> loadLabels(const std::string& labelsFilePath);

}  // namespace inference
}  // namespace robocar
