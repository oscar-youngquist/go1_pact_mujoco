/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <assert.h>
#include <type_traits>
#include <Eigen/Dense>

namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}

// linear interpolation between x and y for xq_query
template <typename T, typename T2>
Eigen::Matrix<typename T::Scalar, T2::ColsAtCompileTime, 1 >
interp1(const Eigen::MatrixBase<T>& x, const Eigen::MatrixBase<T2>& y, const typename T::Scalar& xq_query) {
  static_assert(T::ColsAtCompileTime == 1,
                "x must be a vector");
  
  static_assert(static_cast<int>(T::RowsAtCompileTime) == static_cast<int>(T2::RowsAtCompileTime),
                "y must have as many columns as x has rows"); // interpolation happense column-wise

    if(x.rows() != y.rows()) {
    throw std::runtime_error("x and y must have the same sizes");
  }
  // return the first or last value if xq_query is out of bounds
  if (xq_query <= x(0)) {
    return y.row(0);
  }

  if (xq_query >= x(x.rows() - 1)) {
    return y.row(x.rows() - 1);
  }

  int i = 0;
  while (xq_query > x(i)) {
    i++;
  }

  assert(i > 0 && i < x.rows());

  typename T::Scalar x0 = x(i - 1);
  typename T::Scalar x1 = x(i);

  Eigen::Matrix<typename T::Scalar, T2::ColsAtCompileTime, 1 > y_query;
  y_query = 
  y.row(i - 1) + (y.row(i) - y.row(i - 1)) * (xq_query - x0) / (x1 - x0);

  return y_query;
}


}  // namespace Interpolate

#endif  // PROJECT_INTERPOLATION_H
