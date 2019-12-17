#pragma once

#include <vector>

#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>

namespace robocar {
namespace math {

/**
 * Polynomial fit functions
 */
void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst,
             int order);
std::vector<double> polyfit(const std::vector<cv::Point>& src, int order);

}  // namespace math
}  // namespace robocar
