#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
// #include "Constants.h"
#include "Kinematics.h"
#include "RobotDimensions.h"

 

namespace Kinematics
{

  Eigen::Matrix4d _RLeg_T[8];
  Eigen::Matrix4d _RLeg_ABase0,_RLeg_Ryp2,_RLeg_A7RFoot;
  Eigen::Matrix4d _LLeg_T[8];
  Eigen::Matrix4d _LLeg_ABase0,_LLeg_Ryp2,_LLeg_A7LFoot;
}


template<typename _Matrix_Type_>
_Matrix_Type_ Kinematics::pseudoInverse(const _Matrix_Type_ &a, double epsilon )//= std::numeric_limits<double>::epsilon()
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


 Eigen::Matrix4d Kinematics::init_transform(double a,double al,double d,double t)
{
  Eigen::Matrix4d T=Eigen::Matrix4d::Identity();

////Modified D-H
  T(0,0)=cos(t);
  T(0,1)=-sin(t);
  T(0,2)=0;
  T(0,3)=a;
  T(1,0)=sin(t)*cos(al);
  T(1,1)=cos(t)*cos(al);
  T(1,2)=-sin(al);
  T(1,3)=-sin(al)*d;
  T(2,0)=sin(t)*sin(al);
  T(2,1)=cos(t)*sin(al);
  T(2,2)=cos(al);
  T(2,3)=cos(al)*d;
  return T;
}


void Kinematics::get_LLeg_transform(Eigen::Matrix<double,6,1> t)
{
  double t1,t2,t3,t4,t5,t6;

  if(t.size() != 6)
  {
    ROS_ERROR("The number of joint is wrong.");
    return ;
  }


  t1=t(0);
  t2=t(1);
  t3=t(2);
  t4=t(3);
  t5=t(4);
  t6=t(5);

////Modified D-H
  _LLeg_ABase0<<1,0,0,xHipOffset,
     0,1,0,yHipOffset,
     0,0,1,-zHipOffset,
     0,0,0,1;
  _LLeg_T[1]=init_transform(0,0,0,t1-pi/2);//T01
  _LLeg_T[2]=init_transform(0,pi/2,0,t2-pi/2);//T12
  _LLeg_T[3]=init_transform(0,pi/2,0,t3+pi);//T23
  _LLeg_T[4]=init_transform(-upperLegLength,0,0,t4);//T34
  _LLeg_T[5]=init_transform(-lowerLegLength,0,0,t5);//T45
  _LLeg_T[6]=init_transform(0,pi/2,0,t6);//T56
  _LLeg_Ryp2<< 0, 0, 1, 0,
    0, 1, 0, 0,
    -1, 0, 0, 0,
    0, 0, 0, 1;   //Ry(pi/2)
  _LLeg_A7LFoot<< 1,0,0,footXOffset,
       0,1,0,footYOffset,
       0,0,1,-footHeight,
       0,0,0,1;
}

void Kinematics::get_RLeg_transform(Eigen::Matrix<double,6,1> t)
{
  double t1,t2,t3,t4,t5,t6;

  if(t.size() != 6)
  {
    ROS_ERROR("The number of joint is wrong.");
    return ;
  }


  t1=t(0);
  t2=t(1);
  t3=t(2);
  t4=t(3);
  t5=t(4);
  t6=t(5);

////Modified D-H
  _RLeg_ABase0<<1,0,0,xHipOffset,
     0,1,0,-yHipOffset,
     0,0,1,-zHipOffset,
     0,0,0,1;
  _RLeg_T[1]=init_transform(0,0,0,t1-pi/2);//T01
  _RLeg_T[2]=init_transform(0,pi/2,0,t2-pi/2);//T12
  _RLeg_T[3]=init_transform(0,pi/2,0,t3+pi);//T23
  _RLeg_T[4]=init_transform(-upperLegLength,0,0,t4);//T34
  _RLeg_T[5]=init_transform(-lowerLegLength,0,0,t5);//T45
  _RLeg_T[6]=init_transform(0,pi/2,0,t6);//T56
  _RLeg_Ryp2<< 0, 0, 1, 0,
    0, 1, 0, 0,
    -1, 0, 0, 0,
    0, 0, 0, 1;   //Ry(pi/2)
  _RLeg_A7RFoot<< 1,0,0,footXOffset,
       0,1,0,-footYOffset,
       0,0,1,-footHeight,
       0,0,0,1;
}



Eigen::Matrix4d Kinematics::forward_kinematics(Eigen::Matrix<double,6,1> t,Chains::Chain chain)
{
  Eigen::Matrix4d p;

  if(chain == Chains::leftLeg)
  {
    get_LLeg_transform(t);
    p=_LLeg_ABase0;
    for(unsigned int i = 1 ; i < 7 ; ++i)
      p *= _LLeg_T[i];
    p =p*_LLeg_Ryp2*_LLeg_A7LFoot;
  }
  else if(chain == Chains::rightLeg)
  {
    get_RLeg_transform(t);
    p=_RLeg_ABase0;
    for(unsigned int i = 1 ; i < 7 ; ++i)
      p *= _RLeg_T[i];
    p =p*_RLeg_Ryp2*_RLeg_A7RFoot;
  }

  return p;
}

//Modified D-H
Eigen::Matrix<double,6,6> Kinematics::calc_jacobi(Eigen::Matrix<double,6,1> t,Chains::Chain chain)
{
  Eigen::Vector3d z[6];
  Eigen::Vector3d p_end_effector;
  Eigen::Vector3d p[6];
  Eigen::Vector3d delta_vec;
  Eigen::Vector3d d_rev[6];
  Eigen::Matrix4d transf_matrix[7];
  Eigen::Matrix<double,6,6> J;


  if(chain == Chains::leftLeg)
  {
    get_LLeg_transform(t);

    for(unsigned int i = 0 ; i < 7 ; ++i)
      transf_matrix[i] = _LLeg_ABase0;

    for(unsigned int i = 0 ; i < 6 ; ++i)
      for(unsigned int j = 0 ; j < i+1 ; ++j)
        transf_matrix[i] *= _LLeg_T[j+1];

    transf_matrix[6]=transf_matrix[5]*_LLeg_Ryp2*_LLeg_A7LFoot;
  }
  else if(chain == Chains::rightLeg)
  {
    get_RLeg_transform(t);

    for(unsigned int i = 0 ; i < 7 ; ++i)
      transf_matrix[i] = _RLeg_ABase0;

    for(unsigned int i = 0 ; i < 6 ; ++i)
      for(unsigned int j = 0 ; j < i+1 ; ++j)
        transf_matrix[i] *= _RLeg_T[j+1];

    transf_matrix[6]=transf_matrix[5]*_RLeg_Ryp2*_RLeg_A7RFoot;
  }

  //Position of end effector
  p_end_effector<< transf_matrix[6](0,3) , transf_matrix[6](1,3) , transf_matrix[6](2,3);

  for(unsigned int i = 0 ; i < 6 ; ++i)
  {
    z[i] << transf_matrix[i](0,2) , transf_matrix[i](1,2) , transf_matrix[i](2,2);
    p[i] << transf_matrix[i](0,3) , transf_matrix[i](1,3) , transf_matrix[i](2,3);

    delta_vec = p_end_effector - p[i];
    d_rev[i] = z[i].cross(delta_vec);
    J(0,i)=d_rev[i](0);
    J(1,i)=d_rev[i](1);
    J(2,i)=d_rev[i](2);
    J(3,i)=z[i](0);
    J(4,i)=z[i](1);
    J(5,i)=z[i](2);
  }

  return J;
}


 Eigen::Matrix<double,6,1> Kinematics::calc_del_theta_DLS(Eigen::Matrix<double,6,1> theta,Eigen::Matrix<double,6,1> del_p,Chains::Chain chain)
{
  Eigen::Matrix<double,6,1> del_theta;
  Eigen::MatrixXd J;
  double nu=0.001;

  if(chain == Chains::leftLeg)
    J=calc_jacobi(theta,Chains::leftLeg);
  else if(chain == Chains::rightLeg)
    J=calc_jacobi(theta,Chains::rightLeg);


  //Algorithm
  //TODO optimization needed

  Eigen::MatrixXd one = J;
  Eigen::MatrixXd two = J * J.transpose();
  Eigen::MatrixXd id = Eigen::MatrixXd::Identity(two.rows() , two.cols());
  id = (nu*nu) * id;

  Eigen::MatrixXd result = two + id ;
  Eigen::MatrixXd result_out;

  result_out=pseudoInverse(result);

  result_out = J.transpose() * result_out;

  del_theta =  result_out *del_p;

  return del_theta;

}

Eigen::Matrix<double,6,1> Kinematics::cal_del_pos(Eigen::Matrix4d c_pos,Eigen::Matrix4d d_pos)
{
  Eigen::Matrix<double,6,1> pos_vec;
  Eigen::Matrix<double,3,1> c_n,c_o,c_a,d_n,d_o,d_a,temp;

  for(unsigned i=0;i<3;i++)
  {
    c_n(i)=c_pos(i,0);
    c_o(i)=c_pos(i,1);
    c_a(i)=c_pos(i,2);

    d_n(i)=d_pos(i,0);
    d_o(i)=d_pos(i,1);
    d_a(i)=d_pos(i,2);
  }

  temp=(c_n.cross(d_n)+c_o.cross(d_o)+c_a.cross(d_a))/2;

  pos_vec(0)=d_pos(0,3)-c_pos(0,3);
  pos_vec(1)=d_pos(1,3)-c_pos(1,3);
  pos_vec(2)=d_pos(2,3)-c_pos(2,3);
  pos_vec(3)=temp(0);
  pos_vec(4)=temp(1);
  pos_vec(5)=temp(2);

  return pos_vec;
}


bool Kinematics::inverse_kinematics(Eigen::Matrix4d p,Chains::Chain chain,Eigen::Matrix<double,6,1>& theta1)
{
  Eigen::Matrix<double,6,1> theta = theta1*Util::TO_RADIAN;

  Eigen::Matrix<double,6,6> inverse;
  // Eigen::JacobiSVD<Eigen::MatrixXf> solver;
  Eigen::MatrixXd J,J_pinv;
  Eigen::Matrix4d pos;

  // theta<< 0,0,0,0,0,0;   //初始猜测

  if(chain == Chains::leftLeg)
    pos=forward_kinematics(theta,Chains::leftLeg);
  else if(chain == Chains::rightLeg)
    pos=forward_kinematics(theta,Chains::rightLeg);

  if(p.size() != 16)
  {
    ROS_ERROR("The number of transform matrix is wrong.");
    exit(-1) ;
  }

  Eigen::Matrix4d dst_pos;
  Eigen::Matrix<double,6,1> del_theta,del_pos;

  dst_pos=p;

  del_pos=cal_del_pos(pos,dst_pos);

  int count=0;

  // double time1=ros::Time::now().toSec();
  while(del_pos.norm()>0.1)
  {
    count++;
    if(chain == Chains::leftLeg)
      del_theta=calc_del_theta_DLS(theta,del_pos,Chains::leftLeg);
    else if(chain == Chains::rightLeg)
      del_theta=calc_del_theta_DLS(theta,del_pos,Chains::rightLeg);

    theta+=del_theta;

    if(chain == Chains::leftLeg)
      pos=forward_kinematics(theta,Chains::leftLeg);
    else if(chain == Chains::rightLeg)
      pos=forward_kinematics(theta,Chains::rightLeg);
      del_pos=cal_del_pos(pos,dst_pos);
  //  std::cout<<"del_theta:\n" <<  del_theta <<std::endl;
    // std::cout<<"pos:\n" <<  pos <<std::endl;
  //  std::cout<<"del_pos:\n" <<  del_pos <<std::endl;
    // if(count==2000){double time2=ros::Time::now().toSec();std::cout<< " 2000 time:" << time2-time1 <<std::endl;std::cout<<"pos:\n" <<  pos <<std::endl;return false;}
    if(count==2000){std::cout<< " 2000 time:" <<std::endl;std::cout<<"pos:\n" <<  pos <<std::endl;return false;}
  }
  // double time2=ros::Time::now().toSec();
  // std::cout<< count << "  time:" << (time2-time1)*1000 <<std::endl;
  // std::cout<<"pos:\n" <<  pos <<std::endl;

  theta1 = theta*Util::TO_DEGREE;
  
  return true;
}



bool Kinematics::inverse_kinematics1(Eigen::Matrix4d p,Chains::Chain chain,Eigen::Matrix<double,6,1>& theta)
{
   if(chain == Chains::leftLeg)
    theta=LLeg_inverse_kinematics(p);
  else if(chain == Chains::rightLeg)
    theta=RLeg_inverse_kinematics(p);

  return true;
}


Eigen::Matrix<double,6,1> Kinematics::RLeg_inverse_kinematics(Eigen::Matrix4d p)
{
  double r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz;
  double a,b,c,d;
  double t1,t2,t3,t4,t5,t6;
  Eigen::Matrix<double,6,1> theta;

  r11=p(0,0);
  r12=p(0,1);
  r13=p(0,2);
  px=p(0,3);
  r21=p(1,0);
  r22=p(1,1);
  r23=p(1,2);
  py=p(1,3);
  r31=p(2,0);
  r32=p(2,1);
  r33=p(2,2);
  pz=p(2,3);

///t4
  double px1,py1,pz1;
  px1=- r23*(yHipOffset + py + footHeight*r23 - footXOffset*r21 + footYOffset*r22) - r33*(zHipOffset + pz + footHeight*r33 - footXOffset*r31 + footYOffset*r32) - r13*(px - xHipOffset+ footHeight*r13 - footXOffset*r11 + footYOffset*r12);
  py1=- r22*(yHipOffset + py + footHeight*r23 - footXOffset*r21 + footYOffset*r22) - r32*(zHipOffset + pz + footHeight*r33 - footXOffset*r31 + footYOffset*r32) - r12*(px - xHipOffset+ footHeight*r13 - footXOffset*r11 + footYOffset*r12);
  pz1=r21*(yHipOffset + py + footHeight*r23 - footXOffset*r21 + footYOffset*r22) + r31*(zHipOffset + pz + footHeight*r33 - footXOffset*r31 + footYOffset*r32) + r11*(px - xHipOffset+ footHeight*r13 - footXOffset*r11 + footYOffset*r12);

  a=(px1*px1+py1*py1+pz1*pz1-upperLegLength*upperLegLength-lowerLegLength*lowerLegLength)/(2*upperLegLength*lowerLegLength);
  if(a>1){std::cout<<"Ra:\n" <<  a << std::endl;a=1;}// a=1;
  if(a<-1) {std::cout<<"Ra:\n" <<  a << std::endl;a=-1;}//a=-1;
  t4=acos(a);

  double c4,s4;
  c4=cos(t4);
  s4=sin(t4);

///t5
  double t51,t52;

  a=upperLegLength*s4;
  b=upperLegLength*c4+lowerLegLength;
  c=pz1;
  d=a*a+b*b-c*c;
  t51=atan2(b,a)+atan2(sqrt(d),c);
  t52=atan2(b,a)-atan2(sqrt(d),c);
  t5=t52;

  double c5,s5,c45;
  c5=cos(t5);
  s5=sin(t5);
  c45=cos(t4+t5);

///t6

  if(upperLegLength*c45+lowerLegLength*c5 != 0)
    t6=atan2(-py1,px1);


  double c6,s6;
  c6=cos(t6);
  s6=sin(t6);

///t2,t3,t1
  double r13_2,r23_2,r33_2,r31_2,r32_2;
  r13_2=r12*c6 + r13*s6;
  r23_2=r22*c6 + r23*s6;
  r33_2=r32*c6 + r33*s6;
  r31_2=c4*(c5*(r33*c6 - r32*s6) - r31*s5) - s4*(s5*(r33*c6 - r32*s6) + r31*c5);
  r32_2=c4*(s5*(r33*c6 - r32*s6) + r31*c5) + s4*(c5*(r33*c6 - r32*s6) - r31*s5);

  t2=asin(-r33_2);
  if(cos(t2) != 0)
  {
    t1=atan2(-r13_2,r23_2);
    t3=atan2(-r32_2,r31_2);
  }


  theta(0)=t1;
  theta(1)=t2;
  theta(2)=t3;
  theta(3)=t4;
  theta(4)=t5;
  theta(5)=t6;


  return theta;


}


Eigen::Matrix<double,6,1> Kinematics::LLeg_inverse_kinematics(Eigen::Matrix4d p)
{
  double r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz;
  double a,b,c,d;
  double t1,t2,t3,t4,t5,t6;
  Eigen::Matrix<double,6,1> theta;

  r11=p(0,0);
  r12=p(0,1);
  r13=p(0,2);
  px=p(0,3);
  r21=p(1,0);
  r22=p(1,1);
  r23=p(1,2);
  py=p(1,3);
  r31=p(2,0);
  r32=p(2,1);
  r33=p(2,2);
  pz=p(2,3);

///t4
  double px1,py1,pz1;
  px1=- r23*(-yHipOffset + py + footHeight*r23 - footXOffset*r21 - footYOffset*r22) - r33*(zHipOffset + pz + footHeight*r33 - footXOffset*r31 - footYOffset*r32) - r13*(px - xHipOffset+ footHeight*r13 - footXOffset*r11 - footYOffset*r12);
  py1=- r22*(-yHipOffset + py + footHeight*r23 - footXOffset*r21 - footYOffset*r22) - r32*(zHipOffset + pz + footHeight*r33 - footXOffset*r31 - footYOffset*r32) - r12*(px - xHipOffset+ footHeight*r13 - footXOffset*r11 - footYOffset*r12);
  pz1=r21*(-yHipOffset + py + footHeight*r23 - footXOffset*r21 - footYOffset*r22) + r31*(zHipOffset + pz + footHeight*r33 - footXOffset*r31 - footYOffset*r32) + r11*(px - xHipOffset+ footHeight*r13 - footXOffset*r11 - footYOffset*r12);


  a=(px1*px1+py1*py1+pz1*pz1-upperLegLength*upperLegLength-lowerLegLength*lowerLegLength)/(2*upperLegLength*lowerLegLength);
  if(a>1) {std::cout<<"La:\n" <<  a << std::endl;a=1;}// a=1;
  if(a<-1) {std::cout<<"La:\n" <<  a << std::endl;a=-1;}//a=-1;
  t4=acos(a);

  double c4,s4;
  c4=cos(t4);
  s4=sin(t4);

///t5
  double t51,t52;

  a=upperLegLength*s4;
  b=upperLegLength*c4+lowerLegLength;
  c=pz1;
  d=a*a+b*b-c*c;
  t51=atan2(b,a)+atan2(sqrt(d),c);
  t52=atan2(b,a)-atan2(sqrt(d),c);
  t5=t52;

  double c5,s5,c45;
  c5=cos(t5);
  s5=sin(t5);
  c45=cos(t4+t5);

///t6

  if(upperLegLength*c45+lowerLegLength*c5 != 0)
    t6=atan2(-py1,px1);


  double c6,s6;
  c6=cos(t6);
  s6=sin(t6);

///t2,t3,t1
  double r13_2,r23_2,r33_2,r31_2,r32_2;
  r13_2=r12*c6 + r13*s6;
  r23_2=r22*c6 + r23*s6;
  r33_2=r32*c6 + r33*s6;
  r31_2=c4*(c5*(r33*c6 - r32*s6) - r31*s5) - s4*(s5*(r33*c6 - r32*s6) + r31*c5);
  r32_2=c4*(s5*(r33*c6 - r32*s6) + r31*c5) + s4*(c5*(r33*c6 - r32*s6) - r31*s5);

  t2=asin(-r33_2);
  if(cos(t2) != 0)
  {
    t1=atan2(-r13_2,r23_2);
    t3=atan2(-r32_2,r31_2);
  }


  theta(0)=t1;
  theta(1)=t2;
  theta(2)=t3;
  theta(3)=t4;
  theta(4)=t5;
  theta(5)=t6;


  return theta;


}













