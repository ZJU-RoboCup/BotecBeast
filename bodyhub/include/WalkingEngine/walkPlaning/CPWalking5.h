
#ifndef CPWalking5_H
#define CPWalking5_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include "TalosRobot.h"

#include "SimControll.h"

#include "Kalman.h"

using namespace ljnoid;

// namespace Eigen
// {
//     typedef Matrix<double, 6, 1> Vector6d;
// }

namespace GaitManager {

template <class V>
constexpr V sqr(const V &a) {
  return a * a;
}

class CPWalking5 {
 public:
  CPWalking5(uint8_t mode = 0);
  virtual ~CPWalking5();

  void setRunMode(uint8_t mode = 0);
  void setTimeStep(double_t t);

  enum StanceStatus {
    LEFT_STANCE,
    RIGHT_STANCE,
    DOUBLE_STANCE,
    LEFT_FOOT_START,
    RIGHT_FOOT_START,
  } currentStanceStatus,
      startStance;

  enum FootprintStatus {
    START,
    NORMAL,
    STOP,
  } footprintStatus;

  struct Footprint{
    Eigen::Matrix<double, 5, 3> fPrint;
    double sT;
    FootprintStatus fprintStatus;
    StanceStatus stanceSta;
  };

  enum StateOfContact {
    DoubleContact = 0,
    LeftContact = -1,
    RightContact = 1,
    UnstableContact = 2
  } ContactState,
      LastContactState;

  enum PhaseOfStep {
    DoubleSupport = 0,
    LeftStance = -1,
    RightStance = 1
  } StepPhase,
      LastStepPhase;
  enum ChangeOfPhase { NoChange, DtoL, DtoR, LtoD, RtoD } PhaseChange;

  int FSR_L[4], FSR_R[4];

  // void GroundContactDetect();
  // void updateStepPhase();
  // init simulator
  void initSquat();  // init jointValue and torsoHeight,torsoHeightWalk

  void constructOneStep(Eigen::MatrixXd &fPrint, FootprintStatus fprintStatus,
                        StanceStatus stanceSta, Eigen::Vector3d delxytheta,
                        Eigen::Vector2d leftFootp, Eigen::Vector2d rightFootp,
                        bool conflict);
  void nextFootprintPostion(StanceStatus stanceSta, Eigen::Vector3d delxytheta,
                            Eigen::Vector2d &leftFootp,
                            Eigen::Vector2d &rightFootp);

  void commandRosInit();

  void sendOneStepRequest();
  void receiveOneStepCommand(Eigen::Vector3d delxytheta);
  void planStopStep();
  void updateFiveFprintFrame(Eigen::MatrixXd fPrint, Eigen::Vector3d &fPlast);
  void receiveFootprint(Eigen::MatrixXd fPrint, double sT,
                        FootprintStatus fprintStatus,
                        StanceStatus stanceSta);  // receive footprint(and store)
                                                  // in Footprint(struct queue)

  void run();
  void updateThreeStep();  // update trajectory param according Footprint(struct)
                           // queue
  void updateThreeStep(
      Eigen::MatrixXd fPrint, double sT, FootprintStatus fprintStatus,
      StanceStatus stanceSta);  // update three steps after every step
  void renewComAtStanceConvert(
      Eigen::Vector3d nextfPrint, Eigen::Vector3d &com,
      Eigen::Vector3d &comv);  // renew com in current foot to next foot
  // void convertTrajToNextStep(Eigen::Vector3d nextfPrint, Eigen::MatrixXd
  // traj, int trajNum, Eigen::MatrixXd &trajInNextStep);
  void planCopEnd(Eigen::MatrixXd fPrint, FootprintStatus fprintStatus);
  // void planCopEnd_NoDS(Eigen::MatrixXd fPrint, FootprintStatus fprintStatus);
  int computeTrajNum(double stepTime, FootprintStatus fprintStatus);

  double computeCpCoef(int j, double Ti, Eigen::Matrix2d &si,
                       Eigen::Matrix2d &expi);
  void planCpTraj(Eigen::MatrixXd prefEnd, double stepTime,
                  FootprintStatus fprintStatus);  // derive capture point
                                                  // reference trajectory from
                                                  // cop reference trajectory

  void planComTraj(Eigen::MatrixXd cp_ref, double stepTime,
                   FootprintStatus fprintStatus);
  void convertTrajToWorld(Eigen::MatrixXd traj, Eigen::MatrixXd &trajInWorld,
                          Eigen::Vector3d fpInWorld);
  void convertCOMvToWorld(Eigen::MatrixXd COMv, Eigen::MatrixXd &COMvInWorld,
                          Eigen::Vector3d fpInWorld);

  void computeFootprintInWorld(Eigen::MatrixXd fPrint);  // for debug

  static CPWalking5 *GetInstance() { return m_UniqueInstance; }

  uint8_t runMode;
  double timeStep;
  Eigen::VectorXd jointValue;
  Eigen::VectorXd lastJointValue;
  Eigen::VectorXd jointVelocity;
  Eigen::VectorXd measuredJointValue;
  Eigen::VectorXd lastMeasuredJointValue;
  Eigen::VectorXd measuredJointVelocity;

  enum jointId {
    LLEG_JOINT_START = 0,
    LLEG_JOINT_END = 6,
    LLEG_JOINT_NUM = 6,

    RLEG_JOINT_START = 6,
    RLEG_JOINT_END = 12,
    RLEG_JOINT_NUM = 6,

    LARM_JOINT_START = 12,
    LARM_JOINT_END = 15,
    LARM_JOINT_NUM = 3,

    RARM_JOINT_START = 15,
    RARM_JOINT_END = 18,
    RARM_JOINT_NUM = 3,

    JOINT_NUM = 12,

    FLOATING_FREEDOM_NUM = 6,
    FLOATING_CONFIG_NUM = 7,
    JOINT_FREEDOM_NUM = 18,
    JOINT_CONFIG_NUM = 19,

    LLEG_JOINT_FREEDOM_NUM = 12,  // 6,//
    LLEG_JOINT_CONFIG_NUM = 13,   // 6,//

  };

  // private:

  double TimeCon_x;
  double TimeCon_y;
  double startTimeCon_y;  // special for cp plan at step start
  double normalTimeCon_y;

  double startT, stopT;

  int printNum;
  int pNumRemain;
  Eigen::MatrixXd footPrint;
  Eigen::VectorXd stepT;
  std::vector<Eigen::Vector3d> footPrintOutPut;
  struct CoM_w_frame {
    double CoMx[500];
    double CoMy[500];
    double V0_x[500];  // not using
    double V0_y[500];  // not using
  };                   // 20 steps maxium
  std::vector<CoM_w_frame> plannedCoM_wF_out;

  std::vector<Footprint> fprintStructVec;

  int startTrajCount;
  int stepTrajNum;
  int doubleSupportTrajNum;
  int lastStepTrajNum;
  int lastTrajNum;
  int cpTrajNum;
  int cpTrajNumAll;

  double torsoHeight;      // torso height from forward kinematics
  double torsoHeightWalk;  // torso height at walking
  double Swing_H;
  double gra_g;      // gravity g
  double totalMass;  // robot total mass

  double footSeparate;        // foot seprate at standing for footprint(plan)
  double footSeparateOfWalk;  // foot seprate
  double stepTime;            // one step time for footprint(plan)
  double stepMaxLx;           // one step max in x direction for footprint(plan)
  double stepMaxLy;           // one step max in y direction for footprint(plan)
  double stepMaxPhi;          // one step max phi rotation for footprint(plan)
  int originStepNum;          // default origin step num for footprint(plan)
  double stepMaxArc;          // one step max Arc for circle footprint(plan)

  double
      coplen;  // coplen*2 eaqual cop move length at stance foot in x direction
  double dsRatio;  // double stance ratio of one step time
                   /////////////the follow these all present in world frame at
                   /// the ground
  Eigen::MatrixXd copref;
  Eigen::MatrixXd cpref;
  Eigen::MatrixXd lFootRefTraj;
  Eigen::MatrixXd rFootRefTraj;
  Eigen::MatrixXd nextStepLFootRefTraj;
  Eigen::MatrixXd nextStepRFootRefTraj;
  Eigen::MatrixXd comref;
  Eigen::MatrixXd comvref;

  Eigen::Vector3d comref_stepend;  // openloop control need at the foot convert
  Eigen::Vector3d comvref_stepend;

  Eigen::MatrixXd lFootAttitudeRefTraj;  // use Quaternion represent
  Eigen::MatrixXd rFootAttitudeRefTraj;
  Eigen::MatrixXd comAttitudeRefTraj;

  Eigen::Vector3d comDesir;
  Eigen::Vector3d comvDesir;
  Eigen::Vector3d comaDesir;

  Eigen::MatrixXd pref_end;
  //////////////////

  int countTraj;
  static CPWalking5 *m_UniqueInstance;

  bool planningDone;  // walk planing done flag,to void main.cpp thread to  run
                      // run() in doing walk planing
  bool onWalking;
  bool lastOnWalking;
  bool prepareStop;
  // bool recvOne;

  Eigen::Vector2d cpMeasure;
  Eigen::Vector2d cperr;
  Eigen::Vector2d cperr_old;

  // Eigen::Vector2d copDesir;

  //////In world to debug
  Eigen::Vector2d cpMeasureInWorld;
  Eigen::Vector2d comDesirInWorld;
  Eigen::Vector2d comvDesirInWorld;
  Eigen::Vector2d measuredComInWorld;
  Eigen::Vector2d measuredComVelocityInWorld;
  Eigen::Vector2d estimatedComInWorld;
  Eigen::Vector2d estimatedComVelocityInWorld;
  Eigen::Vector2d cpRefInWorld;
  Eigen::Vector2d copRefInWorld;
  Eigen::Vector2d comRefInWorld;
  Eigen::Vector2d comvRefInWorld;

  // Eigen::Vector2d comMeasure, comvMeasure;

  // InertialDataFilter inertialDataFilter;
  // LIPStateEstimator lipStateEstimator;

  Eigen::Vector3d measuredVelocityLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredVelocityRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedVelocityLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedVelocityRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedCom = Eigen::Vector3d::Zero();

  Eigen::Vector3d measuredLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredComVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d expectedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d expectedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d expectedComVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedComVelocity = Eigen::Vector3d::Zero();

  Eigen::Vector3d lastMeasuredLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastMeasuredRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastEstimatedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastEstimatedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastExpectedLeftToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastExpectedRightToCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastExpectedComVelocity = Eigen::Vector3d::Zero();

  Eigen::Matrix3d covX = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d covY = Eigen::Matrix3d::Identity();

  Eigen::Vector4d observerProcessDeviation = Eigen::Vector4d(
      0.01, 0.01, 0.03, 0.03);  // Eigen::Vector4d(0.01, 0.01, 0.03, 0.03);//
  Eigen::Vector2d observerMeasurementDeviation =
      Eigen::Vector2d(0.02, 0.02);  // Vector2f(0.02,  0.02);//

  Eigen::MatrixXd referFrameInW;  // use to renew reference frame in vrep

  Eigen::Vector3d
      nextFootPrint;  // used to renew com in current foot to next foot

  Eigen::VectorXd
      jointVal;  // lowpass the jointValue command for openloopController

  Footprint lastfpStruct;
  Footprint currentfpStruct;

  std_msgs::Bool reqGaitCommand;
  ros::Publisher reqGaitCommand_pub;
  ros::Subscriber gaitCommand_sub;

  int sendOneTimeCount;
  bool sendOneFlag;

  Eigen::Vector3d fpInWorld;  // stance foot in world

  TalosRobot talosRobot;

  Kalman *leftTorsofilterx;
  Kalman *rightTorsofilterx;
  Kalman *leftTorsofiltery;
  Kalman *rightTorsofiltery;

  bool contactGroundInAdvanced = false;  // for fsr
  int TakeOffCount = 0;                  // for fsr

  bool trigForce = false;

  Eigen::Vector3d lFootRefTrajCmd;  // debug
  Eigen::Vector3d rFootRefTrajCmd;  // debug
};

}  // namespace GaitManager

#endif
