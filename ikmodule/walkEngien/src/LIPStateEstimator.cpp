#include "LIPStateEstimator.h"
// #include "Representations/Sensing/RobotModel.h"
// #include "Tools/Debugging/DebugDrawings3D.h"
// #include "Tools/Math/Rotation.h"

// void LIPStateEstimatorParameters::onRead()
// {
//   positionProcessDeviation.cwiseAbs();
//   velocityProcessDeviation.cwiseAbs();
//   zmpProcessDeviation.cwiseAbs();
//   positionMeasurementDeviation.cwiseAbs();
//   zmpMeasurementDeviation.cwiseAbs();
// }

// LIPStateEstimator::LIPStateEstimator(const InertialData& theInertialData, const RobotModel& theRobotModel):
//   theInertialData(theInertialData), theRobotModel(theRobotModel)
// {}

// void LIPStateEstimator::init(const Array2f& LIPHeights, const Vector2f& leftOrigin, LIPStateEstimatorParameters params)
// {
//   this->params = params;
//   supportFoot = guessSupportFoot(leftOrigin);
//   this->LIPHeights = LIPHeights;
//   origin = leftOrigin;
//   if(supportFoot != SupportFoot::left)
//     origin.y() *= -1;
//   const Vector4f measurement = measure(supportFoot, origin);
//   const Vector6f initMean = (Vector6f() << measurement.head<2>(), Vector2f::Zero(), measurement.tail<2>()).finished();
//   const Vector6f initNoise = (Vector6f() << params.positionProcessDeviation, params.velocityProcessDeviation, params.zmpProcessDeviation).finished();
//   ukf.init(initMean, initNoise.asDiagonal());
// }

// void LIPStateEstimator::update(float timePassed, const Array2f& LIPHeights, const Vector2f& leftOrigin)
// {
//   update(timePassed, LIPHeights, leftOrigin, guessSupportFoot(leftOrigin));
// }

// void LIPStateEstimator::update(float timePassed, const Array2f& LIPHeights, const Vector2f& leftOrigin, const SupportFoot newSupportFoot)
// {
//   this->LIPHeights = LIPHeights;
//   Vector2f newOrigin = leftOrigin;
//   if(newSupportFoot != SupportFoot::left)
//     newOrigin.y() *= -1;
//   const Vector4f measurement = measure(newSupportFoot, newOrigin);
//   if(supportFoot != newSupportFoot)
//   {
//     EstimatedState est = getEstimate();
//     est = convertToOtherFoot(est);
//     ukf.mean.head<2>() = est.com.position;
//     ukf.mean.segment<2>(2) = est.com.velocity;
//     ukf.mean.tail<2>() = est.zmp;
//   }
//   else
//   {
//     ukf.mean.head<2>() = origin + ukf.mean.head<2>() - newOrigin;
//     ukf.mean.tail<2>() = origin + ukf.mean.tail<2>() - newOrigin;
//   }
//   supportFoot = newSupportFoot;
//   origin = newOrigin;

//   auto dynamicMoldel = [&](Vector6f& state)
//   {
//     const Vector2f zmp = state.tail<2>();
//     LIP3D lipState(state.head<2>(), state.segment<2>(2), LIPHeights);
//     lipState.update(timePassed, zmp);
//     state << lipState.position, lipState.velocity, zmp;
//   };
//   auto measurementModel = [&](const Vector6f& state)
//   {
//     return (Vector4f() << state.head<2>(), state.tail<2>()).finished();
//   };

//   const Vector6f dynamicNoise = (Vector6f() << params.positionProcessDeviation, params.velocityProcessDeviation,
//                                  params.zmpProcessDeviation).finished() * timePassed;
//   const Vector4f measurementNoise = (Vector4f() << params.positionMeasurementDeviation, params.zmpMeasurementDeviation).finished() * timePassed;
//   ukf.predict(dynamicMoldel, dynamicNoise.asDiagonal());
//   ukf.update<4>(measurement, measurementModel, measurementNoise.asDiagonal());
// }

// LIPStateEstimator::EstimatedState LIPStateEstimator::getEstimate() const
// {
//   EstimatedState state(LIPHeights);
//   state.com.position << ukf.mean.head<2>();
//   state.com.velocity << ukf.mean.segment<2>(2);
//   state.zmp = ukf.mean.tail<2>();
//   state.supportFoot = supportFoot;
//   state.origin = origin;
//   return state;
// }

// LIPStateEstimator::EstimatedState LIPStateEstimator::convertToOtherFoot(const EstimatedState& state) const
// {
//   EstimatedState otherState(state);
//   otherState.supportFoot = state.supportFoot == SupportFoot::left ? SupportFoot::right : SupportFoot::left;
//   otherState.origin.y() *= -1.f;

//   const Pose3f& soleToTorso = state.supportFoot == SupportFoot::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
//   const Pose3f& otherSoleToTorso = state.supportFoot == SupportFoot::left ? theRobotModel.soleRight : theRobotModel.soleLeft;
//   const Pose3f originToTorso = soleToTorso + Vector3f(state.origin.x(), state.origin.y(), 0.f);
//   const Pose3f otherOriginToTorso = otherSoleToTorso + Vector3f(otherState.origin.x(), otherState.origin.y(), 0.f);

//   //const Quaternionf& torsoToWorld = theInertiaData.orientation;
//   //Pose3f oritinToOtherOrigin = Pose3f(torsoToWorld) * otherOriginToTorso.rotation * otherOriginToTorso.inverse() * originToTorso * originToTorso.rotation.inverse() *= torsoToWorld.inverse();

//   const Pose3f oritinToOtherOrigin = otherOriginToTorso.inverse() * originToTorso;

//   otherState.com.position = (oritinToOtherOrigin * Vector3f(state.com.position.x(), state.com.position.y(), 0.f)).head<2>();
//   otherState.com.velocity = (oritinToOtherOrigin.rotation * Vector3f(state.com.velocity.x(), state.com.velocity.y(), 0.f)).head<2>();
//   otherState.zmp = (oritinToOtherOrigin * Vector3f(state.zmp.x(), state.zmp.y(), 0.f)).head<2>();
//   return otherState;
// }

// LIPStateEstimator::SupportFoot LIPStateEstimator::guessSupportFoot(const Vector2f& leftOrigin) const
// {
//   const Pose3f leftOriginToTorso = theRobotModel.soleLeft + Vector3f(leftOrigin.x(), leftOrigin.y(), 0.f);
//   const Pose3f rightOriginToTorso = theRobotModel.soleRight + Vector3f(leftOrigin.x(), -leftOrigin.y(), 0.f);
//   const Pose3f torsoToWorld(theInertialData.orientation2D);
//   const Pose3f leftOriginToWorld = torsoToWorld * leftOriginToTorso;
//   const Pose3f rightOriginToWorld = torsoToWorld * rightOriginToTorso;

//   if(leftOriginToWorld.translation.z() < rightOriginToWorld.translation.z())
//     return SupportFoot::left;
//   else
//     return SupportFoot::right;
// }

// Vector4f LIPStateEstimator::measure(SupportFoot supportFoot, const Vector2f& LIPOrigin) const
// {
//   const Pose3f& supportFootToTorso = supportFoot == SupportFoot::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
//   const Quaternionf& torsoToWorld = theInertialData.orientation2D;
//   const Pose3f originToTorso = supportFootToTorso + Vector3f(LIPOrigin.x(), LIPOrigin.y(), 0.f);
//   const Vector3f comInOrigin = originToTorso.inverse() * theRobotModel.centerOfMass;
//   const Vector3f com = (torsoToWorld * originToTorso.rotation) * comInOrigin;

//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:Estimate:measuredComHeight", com.z());

//   const Vector2f accInWorld = (torsoToWorld * theInertialData.acc * 1000.f).head<2>();
//   const Vector2f zmp = com.head<2>().array() - (accInWorld.array() / LIP3D(LIPHeights).getK().square());

//   return (Vector4f() << com.head<2>(), zmp).finished();
// }

// void LIPStateEstimator::draw(float footOffset, float forwardingTime) const
// {
//   DEBUG_DRAWING3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", "field")
//   {
//     const Vector4f measurement = measure(supportFoot, origin);
//     const float supportFootSign = static_cast<float>(supportFoot);
//     const Vector3f comToOrigin = (Vector3f() << measurement.head<2>(), LIPHeights.x()).finished();
//     const Vector3f originToSupport = (Vector3f() << origin, 0.f).finished();
//     const Vector3f supportToWorld = (Vector3f() << -origin.x(), supportFootSign * footOffset, 0.f).finished();
//     const Vector3f comInWorld = supportToWorld + originToSupport + comToOrigin;
//     const Vector3f zmp = supportToWorld + originToSupport + (Vector3f() << measurement.tail<2>(), 0.f).finished();

//     SUBCOORDINATES3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", Pose3f(comInWorld), 50, 1);
//     SPHERE3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", comInWorld.x(), comInWorld.y(), comInWorld.z(), 3, ColorRGBA::yellow);
//     CROSS3D("module:ZmpWalkingEngine:LIPStateEstimator:measurement", zmp.x(), zmp.y(), zmp.z(), 3, 5, ColorRGBA::yellow);
//   }

//   DEBUG_DRAWING3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", "field")
//   {
//     const float supportFootSign = static_cast<float>(supportFoot);
//     const Vector3f comToOrigin = (Vector3f() << ukf.mean.head<2>(), LIPHeights.x()).finished();
//     const Vector3f originToSupport = (Vector3f() << origin, 0.f).finished();
//     const Pose3f supportToWorld((Vector3f() << -origin.x(), supportFootSign * footOffset, 0.f).finished());
//     const Vector3f comInWorld = supportToWorld * (originToSupport + comToOrigin);

//     SUBCOORDINATES3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), 50, 1);
//     FOOT3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", supportToWorld, supportFoot == SupportFoot::left, ColorRGBA::orange);
//     const Vector3f comSigma(std::sqrt(ukf.cov(0, 0)), std::sqrt(ukf.cov(1, 1)), 0.001f);
//     ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), comSigma, ColorRGBA(255, 100, 100, 255));
//     ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), Vector3f(comSigma * 2), ColorRGBA(150, 150, 100, 100));
//     ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(comInWorld), Vector3f(comSigma * 3), ColorRGBA(100, 100, 255, 50));

//     Vector3f originToWorld = supportToWorld * originToSupport;
//     const Vector2f zmp = ukf.mean.tail<2>();
//     const Vector3f zmpInWorld = originToWorld + (Vector3f() << zmp, 0.f).finished();
//     LINE3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", originToWorld.x(), originToWorld.y(), originToWorld.z(), zmpInWorld.x(), zmpInWorld.y(), zmpInWorld.z(), 2, ColorRGBA::orange);
//     LINE3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", zmpInWorld.x(), zmpInWorld.y(), zmpInWorld.z(), comInWorld.x(), comInWorld.y(), comInWorld.z(), 2, ColorRGBA::orange);
//     const Vector3f zmpSigma(std::sqrt(ukf.cov(4, 4)), std::sqrt(ukf.cov(5, 5)), 0.001f);
//     ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(zmpInWorld), zmpSigma, ColorRGBA(255, 100, 100, 255));
//     ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(zmpInWorld), Vector3f(zmpSigma * 2), ColorRGBA(150, 150, 100, 100));
//     ELLIPSOID3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", Pose3f(zmpInWorld), Vector3f(zmpSigma * 3), ColorRGBA(100, 100, 255, 50));
//     {
//       const float dt = 0.01f;
//       LIP3D comEstimate(ukf.mean.head<2>() - zmp, ukf.mean.segment<2>(2), LIPHeights);
//       Vector3f futureCom;
//       for(float timeRemaining = 0.3f; timeRemaining >= 0; timeRemaining -= dt)
//       {
//         comEstimate.update(dt);
//         futureCom << comEstimate.position + zmp, LIPHeights.x();
//         const Vector3f futureComInWorld = (supportToWorld * Pose3f(originToSupport + futureCom)).translation;
//         POINT3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", futureComInWorld.x(), futureComInWorld.y(), futureComInWorld.z(), 2, ColorRGBA::black);
//       }
//     }
//     {
//       const float dt = 0.01f;
//       LIP3D comEstimate(ukf.mean.head<2>() - zmp, ukf.mean.segment<2>(2), LIPHeights);
//       Vector3f futureCom;
//       for(float timeRemaining = forwardingTime; timeRemaining >= 0; timeRemaining -= dt)
//       {
//         comEstimate.update(dt);
//         futureCom << comEstimate.position + zmp, LIPHeights.x();
//         const Vector3f futureComInWorld = (supportToWorld * Pose3f(originToSupport + futureCom)).translation;
//         POINT3D("module:ZmpWalkingEngine:LIPStateEstimator:estimate", futureComInWorld.x(), futureComInWorld.y(), futureComInWorld.z(), 2, ColorRGBA::orange);
//       }
//     }
//   }
// }

// void LIPStateEstimator::plot() const
// {
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:posX", ukf.mean(0));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:posY", ukf.mean(1));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:velX", ukf.mean(2));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:velY", ukf.mean(3));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:zmpX", ukf.mean(4));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:estimate:zmpY", ukf.mean(5));

//   Vector4f measurement = measure(supportFoot, origin);
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:posX", measurement(0));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:posY", measurement(1));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:zmpX", measurement(2));
//   PLOT("module:ZmpWalkingEngine:LIPStateEstimator:measurement:zmpY", measurement(3));
// }


#include "Kinematics.h"



LIPStateEstimator::LIPStateEstimator()
{
 
}

void LIPStateEstimator::init(InertialData& inertialData,Eigen::Vector2d cpinit,Eigen::Vector2d copinit)
{
  theInertialData=inertialData;
  const Vector8d measurement = measure(supportFoot);//measurement.head<2>()   measurement.tail<2>()).finished()
  const Vector8d initMean = (Vector8d() << measurement).finished();//cpinit, Eigen::Vector2d::Zero(),cpinit, copinit).finished();
  const Vector8d initNoise = (Vector8d() << params.positionProcessDeviation, params.velocityProcessDeviation,params.cpProcessDeviation, params.zmpProcessDeviation).finished();
  ukf.init(initMean, initNoise.asDiagonal());
}
 
 
void LIPStateEstimator::update(double timePassed, const SupportFoot newSupportFoot,InertialData& inertialData,Eigen::Vector2d cpcmd)
{
  theInertialData=inertialData;
  Vector8d measurement = measure(newSupportFoot);
  if(supportFoot != newSupportFoot)
  {
    double TimeCon_x = std::sqrt(9.81/0.442);     //sqrt(g/H)
    double TimeCon_y = std::sqrt(9.81/0.442);     //sqrt(g/H)
    Eigen::Array2d k;
    k <<TimeCon_x*TimeCon_x,TimeCon_y*TimeCon_y;

    measurement.segment<2>(2)=lastcomvMeasure;
    measurement.segment<2>(4)=comMeasure.array()+lastcomvMeasure.array()/k;


    EstimatedState est = getEstimate();
    est = convertToOtherFoot(est);
    ukf.mean.head<2>() = est.com;
    ukf.mean.segment<2>(2) = est.comVel;
    ukf.mean.segment<2>(4) = est.cp;//cpcmd;//
    ukf.mean.tail<2>() = est.zmp;

    std::cout<< "measurement: " <<  measurement.transpose() <<std::endl;
    std::cout<< "est.com: " <<  est.com.transpose() <<std::endl;
    std::cout<< "est.comVel: " <<  est.comVel.transpose() <<std::endl;
    std::cout<< "est.cp: " <<  est.cp.transpose() <<std::endl;
  }
  // supportFoot = newSupportFoot;

  auto dynamicMoldel = [&](Vector8d& state)
  {
    // const Eigen::Vector2d cp = state.segment<2>(4);
    const Eigen::Vector2d zmp = state.tail<2>();
    EstimatedState est = getEstimate();
    est.update(timePassed,cpcmd);//
    state << est.com, est.comVel,est.cp, zmp;
  };
  auto measurementModel = [&](const Vector8d& state)
  {
    return (Vector8d() << state.segment<8>(0)).finished();
  };

  const Vector8d dynamicNoise = (Vector8d() << params.positionProcessDeviation, params.velocityProcessDeviation, params.cpProcessDeviation,
                                 params.zmpProcessDeviation).finished() * timePassed;
  const Vector8d measurementNoise = (Vector8d() << params.positionMeasurementDeviation,params.velocityMeasurementDeviation,params.cpMeasurementDeviation, params.zmpMeasurementDeviation).finished() * timePassed;
  // std::cout<< "est.comVel: " <<  getEstimate().comVel.transpose() <<std::endl;
  ukf.predict(dynamicMoldel, dynamicNoise.asDiagonal());
  ukf.update<8>(measurement, measurementModel, measurementNoise.asDiagonal());

  lastcomvMeasure=comvMeasure;

  if(supportFoot != newSupportFoot)
  {
    std::cout<< "est.com: " <<  getEstimate().com.transpose() <<std::endl;
    std::cout<< "est.comVel: " <<  getEstimate().comVel.transpose() <<std::endl;
    std::cout<< "est.cp: " <<  getEstimate().cp.transpose() <<std::endl;
  }
  supportFoot = newSupportFoot;
  // std::cout<< "est.com: " <<  getEstimate().com.transpose() <<std::endl;
  // std::cout<< "est.comVel: " <<  getEstimate().comVel.transpose() <<std::endl;


  // double TimeCon_x = std::sqrt(9.81/0.446);     //sqrt(g/H)
  // double TimeCon_y = std::sqrt(9.81/0.446);     //sqrt(g/H)
  // Eigen::MatrixXd k(2,2);
  // k<< TimeCon_x,0.,
  //     0.,TimeCon_y;

  // Eigen::Vector2d position = com;
  // Eigen::Vector2d velocity = comv;
  // // std::cout<< "comVel: " <<  comVel.transpose() <<std::endl;
  // Eigen::Vector2d newPosition = position-k*(position - cpcmd)* timePassed;
  // comv = -k*(position - cpcmd);
  // com=newPosition;

  // com=getEstimate().com;
  // comv=(com-lastcom)/timePassed;
  // lastcom=com;
  // std::cout<< "comv: " <<  comv <<std::endl;
}

LIPStateEstimator::EstimatedState LIPStateEstimator::getEstimate() const
{
  EstimatedState state;
  state.com << ukf.mean.head<2>();
  state.comVel << ukf.mean.segment<2>(2);
  state.cp << ukf.mean.segment<2>(4);
  state.zmp = ukf.mean.tail<2>();
  state.supportFoot = supportFoot;
  return state;
}

LIPStateEstimator::EstimatedState LIPStateEstimator::convertToOtherFoot(const EstimatedState& state) 
{
  EstimatedState otherState(state);
  otherState.supportFoot = state.supportFoot == SupportFoot::left ? SupportFoot::right : SupportFoot::left;

  //根据测量关节角度正解，得测量值
  Eigen::Matrix4d leftMesureH,rightMesureH;
  leftMesureH=Kinematics::forward_kinematics(SimControll::jointAngle.segment(0,6), Chains::leftLeg);
  rightMesureH=Kinematics::forward_kinematics(SimControll::jointAngle.segment(6,6), Chains::rightLeg);

  Eigen::Vector3d posl,posr;
  posl = leftMesureH.block(0,3,3,1); 
  posr = rightMesureH.block(0,3,3,1); 
  leftMesureH.block(0,3,3,1) = posl/1000.; //mm to m
  rightMesureH.block(0,3,3,1) = posr/1000.; 

  Eigen::Vector3d com = (rightMesureH.block(0,0,3,3).inverse()*(-rightMesureH.block(0,3,3,1)));

  otherState.com = state.supportFoot == SupportFoot::left ? state.com+(leftMesureH.inverse()*rightMesureH).block(0,3,2,1) : state.com+(rightMesureH.inverse()*leftMesureH).block(0,3,2,1) ;//com.head<2>();
  otherState.comVel= state.comVel;
  otherState.cp= state.supportFoot == SupportFoot::left ? state.cp+(leftMesureH.inverse()*rightMesureH).block(0,3,2,1) : state.cp+(rightMesureH.inverse()*leftMesureH).block(0,3,2,1) ; 
  otherState.zmp = state.supportFoot == SupportFoot::left ? state.zmp+(leftMesureH.inverse()*rightMesureH).block(0,3,2,1) : state.zmp+(rightMesureH.inverse()*leftMesureH).block(0,3,2,1) ; //Eigen::Vector2d::Zero();
  return otherState;
}


Vector8d LIPStateEstimator::measure(SupportFoot supportFoot) //const
{
  //根据测量关节角度正解，得测量值
  Eigen::Matrix4d leftMesureH,rightMesureH;
  leftMesureH=Kinematics::forward_kinematics(SimControll::jointAngle.segment(0,6), Chains::leftLeg);
  rightMesureH=Kinematics::forward_kinematics(SimControll::jointAngle.segment(6,6), Chains::rightLeg);

  Eigen::Vector3d posl,posr;
  posl = leftMesureH.block(0,3,3,1); 
  posr = rightMesureH.block(0,3,3,1); 
  leftMesureH.block(0,3,3,1) = posl/1000.; //mm to m
  rightMesureH.block(0,3,3,1) = posr/1000.; 


  Eigen::Matrix4d supportFootInTorso = supportFoot == SupportFoot::left ? leftMesureH : rightMesureH;
  Eigen::Quaterniond torsoInWorld = theInertialData.orientation2D;
  Eigen::Vector3d torsoInSupportFoot = supportFootInTorso.block(0,0,3,3).inverse()*(-supportFootInTorso.block(0,3,3,1));
  Eigen::Vector3d com = (torsoInWorld.toRotationMatrix() * supportFootInTorso.block(0,0,3,3)) * torsoInSupportFoot;

  comMeasure=com.head<2>();
  comvMeasure=(comMeasure-lastcomMeasure)/0.02;


  lastcomMeasure=comMeasure;

  double TimeCon_x = std::sqrt(9.81/0.442);     //sqrt(g/H)
  double TimeCon_y = std::sqrt(9.81/0.442);     //sqrt(g/H)
  Eigen::Array2d k;
  k <<TimeCon_x*TimeCon_x,TimeCon_y*TimeCon_y;

  cpMeasure=comMeasure.array()+comvMeasure.array()/k;
  Eigen::Vector2d accInWorld = (torsoInWorld * theInertialData.acc).head<2>();
  // std::cout<< "accInWorld: " <<  theInertialData.orientation3D * theInertialData.acc <<std::endl;
  Eigen::Vector2d zmp = com.head<2>().array() - (accInWorld.array() / k);//Eigen::Vector2d(0.,0.);//
 // std::cout<< "com: " <<  com.head<2>().transpose() <<std::endl;
  return (Vector8d() << comMeasure,comvMeasure,cpMeasure, zmp).finished();
}




























