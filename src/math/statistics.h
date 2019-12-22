#pragma once

#include <iterator>
#include <numeric>

namespace robocar {
namespace math {

/**
 * Calculate the std deviation.
 *
 * @tparam Iterator     The iterator type.
 * @tparam FuncExtract  Function that extracts the data from the list.
 *                      Signature: double(*Iterator);
 *
 * @param begin     The begining of the list.
 * @param end       Past the end of the list.
 * @param extract   The extractor function.
 *
 * @return The standard deviation;
 */
template <class Iterator, class FuncExtract>
double stdDeviation(Iterator begin, Iterator end, FuncExtract&& extract) {
  double sum = std::accumulate(
      begin, end, 0.0, [&extract](const double& partialSum, const auto& cur) {
        return partialSum + extract(cur);
      });
  double mean = sum / std::distance(begin, end);
  double stdDev = std::accumulate(
      begin, end, 0.0,
      [&extract, mean](const double& partialSum, const auto& item) {
        return partialSum + ::pow(extract(item) - mean, 2);
      });
  return ::sqrt(stdDev / std::distance(begin, end));
}
template <class Iterator>
double stdDeviation(Iterator begin, Iterator end) {
  return stdDeviation(begin, end, [](auto item) { return item; });
}

}  // namespace math
}  // namespace robocar
