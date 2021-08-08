/**
   @author
*/

#ifndef UTIL_H
#define UTIL_H

#include <memory>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace Util {

const double PI = 3.14159265358979323846;
const double PI_2 = 1.57079632679489661923;

const double TO_DEGREE = 180.0 / PI;
const double TO_RADIAN = PI / 180.0;

inline double degree(double rad) { return TO_DEGREE * rad; }
inline double radian(double deg) { return TO_RADIAN * deg; }
inline float degree(float rad) { return (float)TO_DEGREE * rad; }
inline float radian(float deg) { return (float)TO_RADIAN * deg; }
inline double radian(int deg) { return TO_RADIAN * deg; }

/*
  Since version 3.2, the behavior of Eigen's eulerAngles function was slightly
  modified; The returned angles are in the ranges [0:pi]x[0:pi]x[-pi:pi]. This
  is not good for using the returned angles to interpolate attitdues. Now our
  own implementation is used for getting R-P-Y angles.
*/
/*
  template<typename Derived>
  inline Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar, 3, 1>
  rpyFromRot(const Eigen::MatrixBase<Derived>& R) {
  Vector3 ea = R.eulerAngles(2, 1, 0);
  return Vector3(ea[2], ea[1], ea[0]); // exchange element order to be our
  conventional one !
  }
*/

Vector3d rpyFromRot(const Matrix3d& R);

Matrix3d rotFromRpy(double r, double p, double y);
Matrix3d rotFromXyzEuler(double r, double p, double y);

std::string str(const Vector3d& v);
std::string str(const Vector3f& v);

void normalizeRotation(Matrix3d& R);

}  // namespace Util

#endif
