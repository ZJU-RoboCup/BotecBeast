#ifndef _Kalman_h_
#define _Kalman_h_

#include <Eigen/Dense>
#include <Eigen/Sparse>

class Kalman {
 public:
  Kalman();
  Kalman(int state, int measure);

  void setQR(double* Q, double* R);
  void update_F_H(Eigen::MatrixXd F, Eigen::MatrixXd H);
  Eigen::MatrixXd updateData(Eigen::MatrixXd z);

  void constructTorsoF();
  void constructTorsoH();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x;

 private:
  int _state, _measure;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _F;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _P;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _H;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _K;
};

#endif
