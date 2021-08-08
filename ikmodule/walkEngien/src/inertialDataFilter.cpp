#include "InertialDataFilter.h"

#include <sstream>




InertialDataFilter::State InertialDataFilter::State::operator+(const Eigen::Vector3d& angleAxis) const
{
  return State(*this) += angleAxis;
}

InertialDataFilter::State& InertialDataFilter::State::operator+=(const Eigen::Vector3d& angleAxis)
{
  orientation = orientation * AngleAxisOpera::unpack(angleAxis);
  return *this;
}

Eigen::Vector3d InertialDataFilter::State::operator-(const State& other) const
{
  return AngleAxisOpera::pack(Eigen::AngleAxisd(other.orientation.inverse() * orientation));
}

void InertialDataFilter::update(InertialData& inertialData)
{
  inertialData.acc =  SimControll::inertialSensorData.acc;
  inertialData.gyro = SimControll::inertialSensorData.gyro;

  const Eigen::Vector3d& chosenAccDeviation =  accDeviation;
  estimate(inertialData.gyro, inertialData.acc, 0.02, gyroDeviation, chosenAccDeviation);

  inertialData.orientation2D = AngleAxisOpera::removeZRotation(ukf.mean.orientation);
  inertialData.orientation3D = ukf.mean.orientation;
  inertialData.angle << AngleAxisOpera::pack(Eigen::AngleAxisd(inertialData.orientation2D)).head<2>(),0.;

  // std::cout<< "angle: " << Util::rpyFromRot(Eigen::AngleAxisd(inertialData.orientation2D).toRotationMatrix()).transpose() <<std::endl;
  // std::cout<< "angle: " << Util::rpyFromRot(inertialData.orientation2D.toRotationMatrix()).transpose() <<std::endl;
  const Eigen::Vector3d angleAxisVec = AngleAxisOpera::pack(Eigen::AngleAxisd(ukf.mean.orientation));

  lastRawAngle = inertialData.angle;
}

void InertialDataFilter::estimate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, double timePassed, const Eigen::Vector3d& gyroDeviation, const Eigen::Vector3d& accDeviation)
{
  auto dynamicModel = [&](State& state)
  {
    state.orientation = state.orientation * AngleAxisOpera::unpack(gyro*timePassed);
    // std::cout<< "angle: " <<std::endl<< AngleAxisOpera::pack(Eigen::AngleAxisd(AngleAxisOpera::unpack(gyro*timePassed))).transpose() <<std::endl;
  };
 
  auto measuremantModelAcc = [&](const State& state)
  {
    const Eigen::Vector3d gVec = Eigen::Vector3d(0., 0., 9.81);//std::cout<< "angle: " <<std::endl<< state.orientation.inverse() * gVec <<std::endl;
    return state.orientation.inverse() * gVec;

  };
// std::cout<< "measuremantModelAcc: " <<std::endl<< measuremantModelAcc<<std::endl;
  const Eigen::Vector3d dynamicNoise = gyroDeviation*std::sqrt(1. / timePassed);
  const Eigen::Vector3d measurementNoise = accDeviation;
  ukf.predict(dynamicModel, dynamicNoise.asDiagonal());
  ukf.update<3>(acc, measuremantModelAcc, measurementNoise.asDiagonal());
// std::cout<< "ukf.mean.orientation: " <<std::endl<< AngleAxisOpera::pack(Eigen::AngleAxisd(ukf.mean.orientation)).transpose()<<std::endl;
  ukf.mean.orientation.coeffs().allFinite();
  ukf.mean.orientation.normalize();

}


void InertialDataFilter::kalmanUpdate(InertialData& inertialData)
{
//   inertialData.acc =  SimControll::inertialSensorData.acc;
//   inertialData.gyro = SimControll::inertialSensorData.gyro;

//    double dt =0.02;   //unit ms

//    double gyroXrate = inertialData.gyro.x()*Util::TO_DEGREE; 
//    gyroXangle += gyroXrate * dt; // Without any filter

//    double gyroYrate = inertialData.gyro.y()*Util::TO_DEGREE; 
//    gyroYangle += gyroYrate *dt; // Without any filter
// // std::cout<< "Groangle: " << gyroXangle <<"  "<<  gyroYangle <<std::endl;
//    double accXval = inertialData.acc.x();
//    double accYval = inertialData.acc.y();
//    double accZval = inertialData.acc.z();

//   // Convert to 360 degrees resolution
//    double accYangle = (std::atan2(accXval, std::sqrt(accYval*accYval+accZval*accZval)))*Util::TO_DEGREE;
//    double accXangle = (std::atan2(accYval, std::sqrt(accXval*accXval+accZval*accZval)))*Util::TO_DEGREE;
// // std::cout<< "angle: " << accXangle <<"  "<<  accYangle <<std::endl;
//   //You might have to tune the filters to get the best values 
//   compAngleX = (0.98 * (compAngleX + (gyroXrate * dt))) + (0.02 * (accXangle));
//   compAngleY = (0.98 * (compAngleY + (gyroYrate * dt))) + (0.02 * (accYangle));

//   kalAngleX  = kalmanX.getAngle(accXangle, gyroXrate, dt);
//   kalAngleY  = kalmanY.getAngle(accYangle, gyroYrate, dt);

//   // inertialData.angle << gyroXangle,gyroYangle,0.;
//   // inertialData.angle << accXangle,accYangle,0.;
//   // inertialData.angle << compAngleX,compAngleY,0.;
//   inertialData.angle << kalAngleX,kalAngleY,0.;

}






Eigen::Vector3d AngleAxisOpera::pack(const Eigen::AngleAxisd& angleAxis)
{
  const double angle = angleAxis.angle();
  if(isZero(angle))
    return Eigen::Vector3d::Zero();
  else
  {
    Eigen::Vector3d axis=angleAxis.axis();
    if(axis.norm() > 1e-9)
      return axis.normalized()*(angle/axis.norm());
    return axis;
  }

}

Eigen::AngleAxisd AngleAxisOpera::unpack(const Eigen::Vector3d& angleAxisVec)
{
  const double angle = angleAxisVec.norm();
  if(isZero(angle))
    return Eigen::AngleAxisd::Identity();
  else
    return Eigen::AngleAxisd(angle, angleAxisVec.normalized());
}

Eigen::Quaterniond AngleAxisOpera::removeZRotation(const Eigen::Quaterniond& rotation)
{
  const Eigen::Vector3d& z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d zR = rotation.inverse() * z;
  const Eigen::Vector3d c = zR.cross(z);
  const double sin = c.norm();
  const double cos = zR.dot(z);
  if(isZero(sin))
    if(cos < 0.f) // 180 degree rotation
      return rotation; // There's no unique decomposition.
    else
      return Eigen::Quaterniond::Identity();
  else
  {
    const double angle = std::atan2(sin, cos);
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, c.normalized()));
  }
}


