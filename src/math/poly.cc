#include "math/poly.h"

#include <opencv2/opencv.hpp>

namespace robocar {
namespace math {

void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst,
             int order) {
  CV_Assert(src_x.rows > 0);
  CV_Assert(src_y.rows > 0);
  CV_Assert(src_x.cols == 1);
  CV_Assert(src_y.cols == 1);
  CV_Assert(dst.cols == 1);
  CV_Assert(dst.rows == (order + 1));
  CV_Assert(order >= 1);

  cv::Mat X;
  X = cv::Mat::zeros(src_x.rows, order + 1, CV_32FC1);
  cv::Mat copy;
  for (int i = 0; i <= order; i++) {
    copy = src_x.clone();
    cv::pow(copy, i, copy);
    cv::Mat M1 = X.col(i);
    copy.col(0).copyTo(M1);
  }
  cv::Mat X_t, X_inv;
  cv::transpose(X, X_t);
  cv::Mat temp = X_t * X;
  cv::Mat temp2;
  cv::invert(temp, temp2);
  cv::Mat temp3 = temp2 * X_t;
  cv::Mat W = temp3 * src_y;
  W.copyTo(dst);
}

std::vector<float> polyfit(const std::vector<cv::Point>& src, int order) {
  cv::Mat src_x(src.size(), 1, CV_32F);
  cv::Mat src_y(src.size(), 1, CV_32F);
  for (int i = 0; i < src.size(); i++) {
    src_x.at<float>(i, 0) = static_cast<float>(src[i].x);
    src_y.at<float>(i, 0) = static_cast<float>(src[i].y);
  }

  cv::Mat result(order + 1, 1, CV_32F);
  polyfit(src_x, src_y, result, order);
  return result;
}

}  // namespace math
}  // namespace robocar
