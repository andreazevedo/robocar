#pragma once

#include <vector>

#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>

namespace robocar {
namespace math {

/**
 * Polynomial fit.
 *
 * @param src_x   Values of x. Must have a single column.
 * @param src_x   Values of y. Must have a single column.
 * @param dest    Output matrix. Must have 1 col and (1 + order) rows.
 *                The coefficients are in increasing power (the index in the
 *                matrix matches the power).
 * @param order   Degree of the fitting polynomial.
 */
void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst,
             int order);

/**
 * Polynomial fit for a vector of points.
 *
 * @param src   The points.
 * @param order Degree of the fitting polynomial.
 *
 * @return      The coefficients. Coefficients are returned in increasing power
 *              (the index in the vector matches the power).
 */
std::vector<float> polyfit(const std::vector<cv::Point>& src, int order);

}  // namespace math
}  // namespace robocar
