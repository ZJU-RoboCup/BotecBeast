#include <iostream>

#include "Kalman.h"



Kalman::Kalman(){


}


Kalman::Kalman(int state,int measure){

    _state = state;
    _measure = measure;

    x.resize(state,1);
    // z.resize(measure,1);

    Q.resize(state, state);
    R.resize(measure, measure);

    _F.resize(state, state);
    _P.resize(state, state);
    _H.resize(measure, state);
    _K.resize(state, measure);


    x.setZero();
    // z.setZero();

    Q.setIdentity();
    R.setIdentity();

    _F.setIdentity();
    _P.setIdentity();
    _H.setIdentity();
    _K.setIdentity();

}

void Kalman::setQR(double* input_q, double* input_r){

    Eigen::VectorXd q(_state);
    for (int i = 0; i < _state; ++i)
        q(i) = input_q[i];
    Eigen::VectorXd r(_measure);
    for (int i = 0; i < _measure; ++i)
        r(i) = input_r[i];

    Q = Eigen::MatrixXd(q.asDiagonal());
    R = Eigen::MatrixXd(r.asDiagonal());



}



void Kalman::update_F_H(Eigen::MatrixXd F, Eigen::MatrixXd H){

    _F = F;
    _H = H;

}



Eigen::MatrixXd Kalman::updateData(Eigen::MatrixXd z) {

    // std::cout<<_P<<std::endl;

    x = _F*x;
    _P = _F*_P*_F.transpose() + Q;

    auto y = _H*x;
    auto dethay = z - y;

    auto _S = _H*_P.inverse()*_H.transpose() + R;
    _K = _P.inverse()*_H.transpose()*_S.inverse();

    x += _K*dethay;

    Eigen::MatrixXd I = _P;
    I.setIdentity();
    _P = (I-_K*_H)*_P.inverse();


    return x;
}




