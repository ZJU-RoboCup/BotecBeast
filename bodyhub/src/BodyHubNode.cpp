#include "bodyhub/bodyhub.h"
#include "controller.h"
#include "imuData.h"
#include "std_msgs/String.h"

#define TO_INT16(a, b)                             \
  ((int16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | \
             ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))

#define BASE_BOARD_ID 200
#define BASE_BOARD_ADDR 24
#define BASE_BOARD_LEN 56

#define GYRO_COEFFICIENT (1000.0 / 32768)
#define ACC_COEFFICIENT (8.0 * 9.8 / 32768)

#define BULK_READ_PRESENT_POSITION_ADDR 36
#define BULK_READ_PRESENT_POSITION_LEN 2
#define RIGHT_FSR_ID 112
#define LEFT_FSR_ID 111
#define FSR_ADDR 90
#define FSR_ADDR_LEN 4
#define MediMotoAlpha 12.80
#define SmalMotoAlpha 18.61

uint8_t dxlIds[DXL_ID_COUNT_MAX];
uint8_t numberOfId = 0;
uint8_t currentControlId = 0;
uint8_t bodyhubState = 0;
std::string stateNewStr;

std::string offsetFile;
std::string InitPoseFile;
std::string sensorNameIDFile;

ImuData torsoImu;
uint8_t imuType = 0;
static double_t dxlGyro[3];
static double_t dxlAcc[3];
static double measuredJointPos[30];
uint8_t armMode = 1;  // 0 为手保持模式
uint8_t failed_ID[SERVO_NUM] = {0};

int WalkJointDirection[SERVO_NUM] = {1,  1, -1, -1, 1, -1, 1, 1, 1, 1, -1,
                                     -1, 1, 1,  1,  1, 1,  1, 1, 1, 1, 1};
float AngleAlpha[SERVO_NUM] = {
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    SmalMotoAlpha, SmalMotoAlpha};

std::vector<double> motoDataValuePre = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double standPos[SERVO_NUM] = {0, 0, 0,   0,   0, 0,  0,  0, 0, 0, 0,
                              0, 0, -75, -10, 0, 75, 10, 0, 0, 0, 0};                                

double servoOffset[SERVO_NUM] = {0};

bool noRobotStand = false;

// 舵机下发数据记录存储
ServoStore_s ServoStore;

std::mutex mtxFSM;

std::mutex mtxJointTrajQueue;
std::queue<sensor_msgs::JointState> jointTrajQueue;

std::queue<bodyhub::JointControlPoint> motoQueue;
std::queue<bodyhub::JointControlPoint> headCtrlQueue;

DynamixelWorkbench dxl_wb;
GaitManager::CPWalking5 *GaitManager::CPWalking5::m_UniqueInstance =
    new CPWalking5;
GaitManager::LIPMWalk mWalk;      // walking
GaitManager::CPWalking5 *cpWalk;  // walking

//创建互斥锁
pthread_mutex_t mtxMo;
pthread_mutex_t mtxHe;
pthread_mutex_t mtxWl;
pthread_mutex_t mtxSL;
pthread_mutex_t mtxReadDxl;


std_msgs::UInt16 budyhubStateMsg;

//话题
ros::Publisher WalkingStatusPub;
ros::Publisher jointPosTargetPub;
ros::Publisher jointPosMeasurePub;
ros::Publisher jointVelTargetPub;
ros::Publisher jointVelMeasurePub;
ros::Publisher contactState_pub;
ros::Publisher stepPhase_pub;
ros::Publisher cpref_pub;
ros::Publisher cpC_pub;
ros::Publisher copm_pub;
ros::Publisher copD_pub;
ros::Publisher copref_pub;
// com
ros::Publisher comRefe_pub;
ros::Publisher comMea_pub;
ros::Publisher comEsti_pub;
// com velocity
ros::Publisher comVRefe_pub;
ros::Publisher comVMea_pub;
ros::Publisher comVEsti_pub;
// left foot
ros::Publisher leftFootRefe_pub;
ros::Publisher leftFootMea_pub;
// right foot
ros::Publisher rightFootRefe_pub;
ros::Publisher rightFootMea_pub;

ros::Publisher Torso_Ppub;
ros::Publisher Torso_Rpub;
ros::Publisher imuDxl_pub;
ros::Publisher imuTorso_pub;
ros::Publisher StatusPub;
ros::Publisher ServoPositionPub;

//服务
ros::ServiceServer imuStateService;
ros::ServiceServer StateService;
ros::ServiceServer MasterIDService;
ros::ServiceServer GetStatusService;
ros::ServiceServer GetJointAngleService;

ros::ServiceServer InstReadValService;
ros::ServiceServer InstWriteValService;
ros::ServiceServer SyncWriteValService;
ros::ServiceServer SetTarPositionValService;
ros::ServiceServer SetTarPositionValAllService;
ros::ServiceServer GetPositionValAllService;

ros::ServiceServer InstReadService;
ros::ServiceServer InstWriteService;
ros::ServiceServer SyncWriteService;
ros::ServiceServer SetLockStateService;
ros::ServiceServer SetLockStateAllService;
ros::ServiceServer GetLockStateAllService;
ros::ServiceServer SetTarPositionService;
ros::ServiceServer SetTarPositionAllService;
ros::ServiceServer GetPositionAllService;

void vectorOut(std::vector<double> &vector_in) {
  /* printf vector double */
  for (unsigned int i = 0; i < vector_in.size(); i++) {
    if (i == vector_in.size() - 1) {
      std::cout << vector_in[i] << std::endl;
    } else
      std::cout << vector_in[i] << ",";
  }
  std::cout << std::endl;
}

void ClearQueue(std::queue<bodyhub::JointControlPoint> &Qtempt) {
  if (!Qtempt.empty()) {
    std::queue<bodyhub::JointControlPoint> empty;
    swap(empty, Qtempt);
  }
}

int getch() {
// 获取按键
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

double Angle2Radian(double angle) { return (angle * PI) / 180; }
double Radian2Angle(double Radian) { return (Radian * 180) / PI; }
double convertValueTOAngle(uint8_t dxlID, int motoValue) {
  return ((motoValue - 2048) / AngleAlpha[dxlID - 1]);
}  // value_to_angle
int convertAngleTOValue(uint8_t dxlID, double motoAngle) {
  return (motoAngle * AngleAlpha[dxlID - 1] + 2048);
}  // angle_to_value

bool isJointError()
{
  if(!ros::ok())
    return true;
  return false;
}

bool WalkingReceiveData(void) {
  /* BulkRead servo Position  FSR*/

  if (!SimControll::simEnable) {
    int32_t present_position[30];
    int32_t present_current[30];
    int32_t present_velocity[30];  // FIXME:
    const char *log = NULL;
    bool result = false;

    uint8_t readCount = 12;
    uint8_t bulkReadID[readCount];
    uint16_t bulkReadAddress[readCount];
    uint16_t bulkReadLength[readCount];
    uint8_t readDataLength = 0;
    int32_t bulkReadData[255];
    int32_t bulkReadWord = 0;

// BulkRead servo Position
#if 0
    for (uint8_t idNum = 0; idNum < readCount; idNum++) {
      bulkReadID[idNum] = dxlIds[idNum];
      bulkReadAddress[idNum] = BULK_READ_PRESENT_POSITION_ADDR;
      bulkReadLength[idNum] = BULK_READ_PRESENT_POSITION_LEN;
      readDataLength += BULK_READ_PRESENT_POSITION_LEN;
    }

    result = dxlBulkRead(bulkReadID, readCount, bulkReadAddress, bulkReadLength, bulkReadData);
    if(result == true)
    {
      for (uint8_t idNum = 0; idNum < readCount; idNum++) {
        bulkReadWord = DXL_MAKEWORD(bulkReadData[idNum * 2], bulkReadData[idNum * 2 + 1]);
        motoDataValuePre[idNum] = present_position[idNum] = bulkReadWord;
        // std::cout << unsigned(dxlIds[idNum]) << "-" << bulkReadWord << "
        // ";//行打印 value
      }

      for (uint8_t idNum = 0; idNum < readCount; idNum++)  // VALUE_TO_DEGREE
      {
        present_position[idNum] =
            present_position[idNum] - servoOffset[idNum];
        measuredJointPos[idNum] =
            (present_position[idNum] - 2048) /
            (AngleAlpha[idNum] * WalkJointDirection[idNum]);
      usleep(10);
    }
#else
    for (uint8_t idNum = 0; idNum < 12; idNum++)  // VALUE_TO_DEGREE
    {
      measuredJointPos[idNum] = mWalk.jointValue[idNum];
    }
#endif
    // BulkRead dxl
    readDataLength = 0;
    uint16_t idx = 0;
    bulkReadID[idx] = BASE_BOARD_ID;
    bulkReadAddress[idx] = BASE_BOARD_ADDR;
    bulkReadLength[idx] = BASE_BOARD_LEN;
    readDataLength += BASE_BOARD_LEN;

    idx++;
    bulkReadID[idx] = LEFT_FSR_ID;
    bulkReadAddress[idx] = FSR_ADDR;
    bulkReadLength[idx] = FSR_ADDR_LEN;
    readDataLength += FSR_ADDR_LEN;

    idx++;
    bulkReadID[idx] = RIGHT_FSR_ID;
    bulkReadAddress[idx] = FSR_ADDR;
    bulkReadLength[idx] = FSR_ADDR_LEN;
    readDataLength += FSR_ADDR_LEN;

    result = dxlBulkRead(bulkReadID, idx + 1, bulkReadAddress, bulkReadLength,
                         bulkReadData);
    if (result == false) {
      return false;
    }

    uint8_t startAddr = BASE_BOARD_ADDR;
    dxlGyro[0] = GYRO_COEFFICIENT * TO_INT16(bulkReadData[38 - startAddr],
                                             bulkReadData[39 - startAddr]);
    dxlGyro[1] = GYRO_COEFFICIENT * TO_INT16(bulkReadData[40 - startAddr],
                                             bulkReadData[41 - startAddr]);
    dxlGyro[2] = GYRO_COEFFICIENT * TO_INT16(bulkReadData[42 - startAddr],
                                             bulkReadData[43 - startAddr]);
    dxlAcc[0] = ACC_COEFFICIENT * TO_INT16(bulkReadData[44 - startAddr],
                                           bulkReadData[45 - startAddr]);
    dxlAcc[1] = ACC_COEFFICIENT * TO_INT16(bulkReadData[46 - startAddr],
                                           bulkReadData[47 - startAddr]);
    dxlAcc[2] = ACC_COEFFICIENT * TO_INT16(bulkReadData[48 - startAddr],
                                           bulkReadData[49 - startAddr]);
    if (imuType == 0) {
      torsoImu.setGyro(dxlGyro[1], dxlGyro[0], -dxlGyro[2]);
      torsoImu.setAcc(dxlAcc[1], dxlAcc[0], -dxlAcc[2]);
    }
#if 0
  std::cout
      <<"gx:"<<gyro[0]
      <<std::setw(10)<<"gy:"<<gyro[1]
      <<std::setw(10)<<"gz:"<<gyro[2]
      <<std::setw(10)<<"ax:"<<acc[0]
      <<std::setw(10)<<"ay:"<<acc[1]
      <<std::setw(10)<<"az:"<<acc[2]
      <<std::setw(5)<<"\r";
#endif

    for (uint8_t i = 0; i < 4; i++) {
      mWalk.FSR_L[i] = bulkReadData[i + 4 + BASE_BOARD_LEN];
      mWalk.FSR_R[i] = bulkReadData[i + BASE_BOARD_LEN];
    }
#if 0
  std::cout << std::setw(5) << mWalk.FSR_L[0] << ":"
      << std::setw(5) << mWalk.FSR_L[1] << ":"
      << std::setw(5) << mWalk.FSR_L[2] << ":"
      << std::setw(5) << mWalk.FSR_L[3] << "  R----L"
      << std::setw(5) << mWalk.FSR_R[0] << ":"
      << std::setw(5) << mWalk.FSR_R[1] << ":"
      << std::setw(5) << mWalk.FSR_R[2] << ":"
      << std::setw(5) << mWalk.FSR_R[3] << "#\r"; // \r  //行打印
#endif
  } else {
    std_msgs::Float64MultiArray jointPosition =
        SimControll::SimRobotData.getJointPostion();

    for (uint8_t i = 0; i < jointPosition.data.size(); i++) {
      measuredJointPos[i] =
          WalkJointDirection[i] * Radian2Angle(jointPosition.data.at(i));
    }

    std_msgs::Float64MultiArray leftFT = SimControll::SimRobotData.getLeftFT();
    std_msgs::Float64MultiArray rightFT =
        SimControll::SimRobotData.getRightFT();
    // ROS_INFO("leftFT: %f\trightFT: %f", leftFT.data.at(5),
    // rightFT.data.at(5));

    if (leftFT.data.at(5) > 0.5)
      leftFT.data[5] = 220;
    else
      leftFT.data[5] = 0;

    if (rightFT.data.at(5) > 0.5)
      rightFT.data[5] = 220;
    else
      rightFT.data[5] = 0;

    for (uint8_t i = 0; i < 4; i++) {
      mWalk.FSR_L[i] = leftFT.data[5];
      mWalk.FSR_R[i] = rightFT.data[5];
    }
  }
}

bool WalkingSendData(void) {
  if (!SimControll::simEnable) {
    const char *log = NULL;
    bool result = false;

    double_t armSwingCoeff = 3;
    uint16_t idIndex = 0;
    uint8_t legIds[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    uint8_t bodyIds[22] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    int32_t motorPos[22] = {0};

    for (uint8_t i = 0; i < numberOfId; i++)  // ANGLE_TO_VALUE
    {
      idIndex = dxlIds[i] - 1;
      if (idIndex < 12)
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1, WalkJointDirection[idIndex] *
                                                 mWalk.jointValue[idIndex]) +
            servoOffset[idIndex];
      else if (idIndex == 12)
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1,
                                -(armSwingCoeff * mWalk.armSwingCount)) +
            servoOffset[idIndex];
      else if (idIndex == 15)
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1,
                                -(armSwingCoeff * mWalk.armSwingCount)) +
            servoOffset[idIndex];
      else if (idIndex == 13)
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else if (idIndex == 14)
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else if (idIndex == 16) 
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else if (idIndex == 17)
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1, standPos[idIndex]) +
            servoOffset[idIndex];
    }
    if (mWalk.RobotState == mWalk.Action_YawAround) {
      motorPos[12] = WalkJointDirection[12] * mWalk.LArm_P * AngleAlpha[12] +
                     2048 + servoOffset[12];
      motorPos[13] = WalkJointDirection[13] * -mWalk.LArm_R * AngleAlpha[13] +
                     2048 + servoOffset[13];
      motorPos[14] =
          WalkJointDirection[14] * -mWalk.LArm_elbow * AngleAlpha[14] + 2048 +
          servoOffset[14];
      motorPos[15] = WalkJointDirection[15] * -mWalk.RArm_P * AngleAlpha[15] +
                     2048 + servoOffset[15];
      motorPos[16] = WalkJointDirection[16] * mWalk.RArm_R * AngleAlpha[16] +
                     2048 + servoOffset[16];
      motorPos[17] =
          WalkJointDirection[17] * mWalk.RArm_elbow * AngleAlpha[17] + 2048 +
          servoOffset[17];

      // ROS_WARN("13:%f, 14:%f, 15:%f, 16:%f, 17:%f,
      // 18:%f",mWalk.LArm_P,-mWalk.LArm_R,-mWalk.LArm_elbow,-mWalk.RArm_P,mWalk.RArm_R,mWalk.RArm_elbow);
    }
    if (armMode == 1) {    
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, bodyIds,
                                22, motorPos, 1, &log);                                                                                                                
    } else {
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, legIds,
                                12, motorPos, 1, &log);                                                       
    }
    if (result == false) {
      ROS_ERROR("fail to syncWrite: %s", log);
    }
  } else {
    uint8_t i = 0;
    std::vector<double> jointCmd;
    std_msgs::Float64MultiArray jointPosition =
        SimControll::SimRobotData.getJointPostion();
    for (i = 0; i < 12; i++) {
      jointCmd.push_back(
          Angle2Radian(WalkJointDirection[i] * mWalk.jointValue[i]));
    }
    if (armMode == 1) {
      for (; i < 18; i++) {
        jointCmd.push_back(0.0);
        if ((i == 12) || (i == 15)) {
          jointCmd[i] =
              Angle2Radian(-1.8 * mWalk.armSwingCount) * WalkJointDirection[i];
        } else {
          jointCmd[i] = Angle2Radian(standPos[i]) * WalkJointDirection[i];
        }
      }
    } else {  // 保持原状态
      for (; i < jointPosition.data.size(); i++) {
        jointCmd.push_back(jointPosition.data[i]);
      }
    }
    SimControll::updateJointCmdQueue(jointCmd);
  }
}


void WalkingStatePublish(void) {
  static bool lastOnWalking = false;
  double_t coeff = 1.0;
  std_msgs::Float64 f64Msg;
  std_msgs::Float64MultiArray f64ArrayMsg;
  std_msgs::Float64MultiArray jointPos;
  jointPos.data.resize(12);

  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] =
        mWalk.measuredJointValue[i] *
        coeff;  // measuredJointPos[i];//measuredJointCurrent[i];//cpWalk->measuredJointValue[i];//
  }
  jointPosMeasurePub.publish(jointPos);
  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] =
        mWalk.jointValue[i] *
        coeff;  // mWalk.jointValue[i];//cpWalk->jointVelocity[i];//
  }
  jointPosTargetPub.publish(jointPos);

  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] =
        cpWalk->measuredJointVelocity[i] *
        coeff;  // measuredJointVel[i];//measuredJointCurrent[i];//cpWalk->measuredJointValue[i];//
  }
  jointVelMeasurePub.publish(jointPos);
  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] = cpWalk->jointVelocity[i] *
                       coeff;  // cpWalk->jointValue[i];//mWalk.jointValue[i];//
  }
  jointVelTargetPub.publish(jointPos);

  if (lastOnWalking != cpWalk->onWalking) {
    lastOnWalking = cpWalk->onWalking;
    f64Msg.data = lastOnWalking;
    WalkingStatusPub.publish(f64Msg);
  }

  f64ArrayMsg.data.resize(6);
  f64ArrayMsg.data[0] = dxlGyro[0];
  f64ArrayMsg.data[1] = dxlGyro[1];
  f64ArrayMsg.data[2] = dxlGyro[2];
  f64ArrayMsg.data[3] = dxlAcc[0];
  f64ArrayMsg.data[4] = dxlAcc[1];
  f64ArrayMsg.data[5] = dxlAcc[2];
  imuDxl_pub.publish(f64ArrayMsg);

  ImuParam_t imuData = torsoImu.getData();
  f64ArrayMsg.data.resize(9);
  f64ArrayMsg.data[0] = imuData.angularVelocity.x;
  f64ArrayMsg.data[1] = imuData.angularVelocity.y;
  f64ArrayMsg.data[2] = imuData.angularVelocity.z;
  f64ArrayMsg.data[3] = imuData.linearAcceleration.x;
  f64ArrayMsg.data[4] = imuData.linearAcceleration.y;
  f64ArrayMsg.data[5] = imuData.linearAcceleration.z;
  f64ArrayMsg.data[6] = imuData.attitude.roll;
  f64ArrayMsg.data[7] = imuData.attitude.pitch;
  f64ArrayMsg.data[8] = imuData.attitude.yaw;
  imuTorso_pub.publish(f64ArrayMsg);

  if (mWalk.StepCountTarget != 0)  //
  {
    std_msgs::Float64 f64Msg;
    std_msgs::Float64MultiArray f64ArrayMsg;

    if (mWalk.StepPhase == GaitManager::LIPMWalk::DoubleSupport)
      f64Msg.data = 0;
    else if (mWalk.StepPhase == GaitManager::LIPMWalk::LeftStance)
      f64Msg.data = -1;
    else if (mWalk.StepPhase == GaitManager::LIPMWalk::RightStance)
      f64Msg.data = 1;
    stepPhase_pub.publish(f64Msg);

    if (mWalk.ContactState == GaitManager::LIPMWalk::DoubleContact)
      f64Msg.data = 0;
    else if (mWalk.ContactState == GaitManager::LIPMWalk::LeftContact)
      f64Msg.data = -0.5;
    else if (mWalk.ContactState == GaitManager::LIPMWalk::RightContact)
      f64Msg.data = 0.5;
    contactState_pub.publish(f64Msg);

    // com
    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.comRefInWorld(0);
    f64ArrayMsg.data[1] = mWalk.comRefInWorld(1);
    f64ArrayMsg.data[2] = mWalk.comRefInWorld(2);
    comRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.measuredComInWorld(0);
    f64ArrayMsg.data[1] = mWalk.measuredComInWorld(1);
    f64ArrayMsg.data[2] = mWalk.measuredComInWorld(2);
    comMea_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.estimatedComInWorld(0);
    f64ArrayMsg.data[1] = mWalk.estimatedComInWorld(1);
    f64ArrayMsg.data[2] = mWalk.estimatedComInWorld(2);
    comEsti_pub.publish(f64ArrayMsg);

    // com velocity
    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.comvRefInWorld(0);
    f64ArrayMsg.data[1] = mWalk.comvRefInWorld(1);
    f64ArrayMsg.data[2] = mWalk.comvRefInWorld(2);
    comVRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.measuredComVelocityInWorld(0);
    f64ArrayMsg.data[1] = mWalk.measuredComVelocityInWorld(1);
    f64ArrayMsg.data[2] = mWalk.measuredComVelocityInWorld(2);
    comVMea_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.estimatedComVelocityInWorld(0);
    f64ArrayMsg.data[1] = mWalk.estimatedComVelocityInWorld(1);
    f64ArrayMsg.data[2] = mWalk.estimatedComVelocityInWorld(2);
    comVEsti_pub.publish(f64ArrayMsg);

    // left foot
    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.PosPara_wF.Lfoot_x;
    f64ArrayMsg.data[1] = mWalk.PosPara_wF.Lfoot_y;
    f64ArrayMsg.data[2] = mWalk.PosPara_wF.Lfoot_z;
    f64ArrayMsg.data[3] = mWalk.PosPara_wF.Lfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.PosPara_wF.Lfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.PosPara_wF.Lfoot_Y * 180.0 / PI;
    leftFootRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.CurrentPosInP.Lfoot_x;
    f64ArrayMsg.data[1] = mWalk.CurrentPosInP.Lfoot_y;
    f64ArrayMsg.data[2] = mWalk.CurrentPosInP.Lfoot_z;
    f64ArrayMsg.data[3] = mWalk.CurrentPosInP.Lfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.CurrentPosInP.Lfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.CurrentPosInP.Lfoot_Y * 180.0 / PI;
    leftFootMea_pub.publish(f64ArrayMsg);

    // right foot
    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.PosPara_wF.Rfoot_x;
    f64ArrayMsg.data[1] = mWalk.PosPara_wF.Rfoot_y;
    f64ArrayMsg.data[2] = mWalk.PosPara_wF.Rfoot_z;
    f64ArrayMsg.data[3] = mWalk.PosPara_wF.Rfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.PosPara_wF.Rfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.PosPara_wF.Rfoot_Y * 180.0 / PI;
    rightFootRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.CurrentPosInP.Rfoot_x;
    f64ArrayMsg.data[1] = mWalk.CurrentPosInP.Rfoot_y;
    f64ArrayMsg.data[2] = mWalk.CurrentPosInP.Rfoot_z;
    f64ArrayMsg.data[3] = mWalk.CurrentPosInP.Rfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.CurrentPosInP.Rfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.CurrentPosInP.Rfoot_Y * 180.0 / PI;
    rightFootMea_pub.publish(f64ArrayMsg);

    // ..
    f64Msg.data = mWalk.CurrentPosInP.Torso_R * 180.0 / PI;
    Torso_Rpub.publish(f64Msg);
    f64Msg.data = mWalk.CurrentPosInP.Torso_P * 180.0 / PI;
    Torso_Ppub.publish(f64Msg);
  }
}

void control_thread_robot() {
  if (pthread_mutex_trylock(&mtxWl) != 0) return;

  WalkingReceiveData();  // instead of  receiveDataFromDxl()
  if (imuType == 0) torsoImu.datafusion();
  for (int i = 0; i < 12; i++)
    mWalk.measuredJointValue[i] = measuredJointPos[i];
  mWalk.measuredJointVelocity =
      (mWalk.measuredJointValue - mWalk.lastMeasuredJointValue) /
      mWalk.timeStep;
  mWalk.lastMeasuredJointValue.segment(0, 12) =
      mWalk.measuredJointValue.segment(0, 12);
  mWalk.talosRobot.measuredmbc.q =
      sVectorToParam(mWalk.talosRobot.talos,
                     mWalk.measuredJointValue.segment(0, 12) * Util::TO_RADIAN);
  mWalk.talosRobot.measuredmbc.alpha = sVectorToDof(
      mWalk.talosRobot.talos,
      mWalk.measuredJointVelocity.segment(0, 12) * Util::TO_RADIAN);

  mWalk.run();
  WalkingSendData();
  WalkingStatePublish();

  pthread_mutex_unlock(&mtxWl);
}

void jointTrajQueuePoll() {
  if (jointTrajQueue.size() > 0) {
    sensor_msgs::JointState jointStateMsg;

    mtxJointTrajQueue.lock();
    jointStateMsg = jointTrajQueue.front();
    jointTrajQueue.pop();
    mtxJointTrajQueue.unlock();

    uint8_t number = jointStateMsg.name.size();
    uint8_t ids[number] = {0};
    int32_t motorPos[number] = {0};

    for (uint8_t i = 0; i < number; i++) {
      ids[i] = std::stoi(jointStateMsg.name[i]);
      motorPos[i] = convertAngleTOValue(ids[i], jointStateMsg.position[i]) +
                    servoOffset[ids[i] - 1];
      ServoStore.angle[ids[i] - 1] = jointStateMsg.position[i];
      ServoStore.value[ids[i] - 1] = motorPos[i];
    }

    bool result = false;
    const char *log = NULL;
    result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, ids, number,
                              motorPos, 1, &log);  //同步写指令                       
    if (result == false) {
      ROS_ERROR("fail to syncWrite: %s", log);
    }

    bodyhub::ServoPositionAngle servoPositionsMsg;
    servoPositionsMsg.angle = ServoStore.angle;
    ServoPositionPub.publish(servoPositionsMsg);
  }
}

void *timerThread(void *ptr) {
  if (SimControll::simEnable) return NULL;
  ROS_INFO("Start 'timerThread' thrand...");

  static struct timespec nextTime;
  struct timespec realTime, lastrealTime;
  double loopTimeout;

  const char *log = NULL;
  bool result = false;
  std::vector<double> ServoRadianStore;

  clock_gettime(CLOCK_MONOTONIC, &nextTime);
  clock_gettime(CLOCK_MONOTONIC, &lastrealTime);

  while (ros::ok()) {
    // timeset
    nextTime.tv_sec = lastrealTime.tv_sec +
                      (lastrealTime.tv_nsec + 10 * 1000000) / 1000000000;
    nextTime.tv_nsec = (lastrealTime.tv_nsec + 10 * 1000000) % 1000000000;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime,
                    NULL);  // start run
    clock_gettime(CLOCK_MONOTONIC, &realTime);
    loopTimeout = ((double)realTime.tv_sec * 1000.0 +
                   (double)realTime.tv_nsec * 0.001 * 0.001) -
                  ((double)lastrealTime.tv_sec * 1000.0 +
                   (double)lastrealTime.tv_nsec * 0.001 * 0.001);
    if (loopTimeout > 10.5)
      ROS_WARN("timerThread() timeout: %f ms", loopTimeout);
    lastrealTime = realTime;

    //== StateEnum::walking
    //下发舵机数据
    if (bodyhubState != StateEnum::walking) {
      if (jointTrajQueue.size() > 0) {
        jointTrajQueuePoll();
      } else if ((motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
        bodyhub::JointControlPoint jntCtrMsg;
        bodyhub::JointControlPoint headCtrMsg;
        bodyhub::ServoPositionAngle servoPositionsMsg;
        int32_t goalPosition[SERVO_NUM];
        uint8_t idArray[SERVO_NUM] = {0};
        uint8_t idCnt = 0;

        if (motoQueue.size() > 0) {
          pthread_mutex_lock(&mtxMo);
          jntCtrMsg = motoQueue.front();
          for (uint8_t idNum = 0;
               idNum < SIZE_LIMIT(jntCtrMsg.positions.size(), SERVO_NUM);
               idNum++) {
            ServoStore.angle[idNum] = jntCtrMsg.positions.at(idNum);
            idArray[idCnt] = idNum + 1;

            goalPosition[idCnt] =
                convertAngleTOValue(idArray[idCnt],
                                    jntCtrMsg.positions.at(idNum)) +
                servoOffset[idArray[idCnt] - 1];
            ServoStore.value[idNum] = goalPosition[idCnt];
            idCnt++;
          }
          motoQueue.pop();
          pthread_mutex_unlock(&mtxMo);
        }
        if (headCtrlQueue.size() > 0) {
          pthread_mutex_lock(&mtxHe);
          if (idCnt > JOINT_SERVO_NUM) idCnt = JOINT_SERVO_NUM;
          headCtrMsg = headCtrlQueue.front();
          for (uint8_t idNum = 0;
               idNum < SIZE_LIMIT(headCtrMsg.positions.size(), HEAD_SERVO_NUM);
               idNum++) {
            ServoStore.angle[idNum + JOINT_SERVO_NUM] =
                headCtrMsg.positions.at(idNum);
            idArray[idCnt] = idCnt + JOINT_SERVO_NUM + 1;  // ID21 ID22

            goalPosition[idCnt] =
                convertAngleTOValue(idArray[idCnt],
                                    headCtrMsg.positions.at(idNum)) +
                servoOffset[idArray[idCnt] - 1];
            ServoStore.value[idNum + JOINT_SERVO_NUM] = goalPosition[idCnt];
            idCnt++;
          }  // ID19 ID20
          headCtrlQueue.pop();
          pthread_mutex_unlock(&mtxHe);
        }

        // #ifdef DEBUG
        //   ROS_INFO("舵机下发：");
        //   vectorOut(ServoStore.angle);
        //   vectorOut(ServoStore.value);
        // #endif

        result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                  idCnt, goalPosition, 1, &log);  //同步写指令                             
        if (result == false) {
          ROS_ERROR("SigHandler()---%s", log);
        }
        ServoRadianStore.clear();
        servoPositionsMsg.angle = ServoStore.angle;
        ServoPositionPub.publish(servoPositionsMsg);
      }
    } else {
      control_thread_robot();
    }
  }

  return NULL;
}



bool ServoBulkRead(uint8_t *bulkReadID, uint8_t readCount, std::string itemName,
                   int32_t *bulkReadData) {
  /* Dynamixels bulkread */
  const char *log = NULL;
  bool result = false;

  dxl_wb.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxl_wb.addBulkReadParam(bulkReadID[idNum], itemName.c_str(), &log);
    if (result == false) ROS_ERROR("ServoBulkRead# %s", log);
  }
  result = dxl_wb.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead$ %s\n", log);
    return false;
  }

  result = dxl_wb.getBulkReadData(&bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead# %s\n", log);
    return false;
  }
  return true;
}

bool dxlBulkRead(uint8_t *bulkReadID, uint8_t readCount,
                 uint16_t *bulkReadAddress, uint16_t *bulkReadLength,
                 int32_t *bulkReadData) {
  const char *log = NULL;
  bool result = false;

  pthread_mutex_lock(&mtxReadDxl);
  dxl_wb.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxl_wb.addBulkReadParam(bulkReadID[idNum], bulkReadAddress[idNum],
                                     bulkReadLength[idNum], &log);
    if (result == false) {
      ROS_ERROR("dxlBulkRead error1: %s", log);
      pthread_mutex_unlock(&mtxReadDxl);
      return false;
    }
  }
  result = dxl_wb.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("dxlBulkRead error2: %s", log);
    pthread_mutex_unlock(&mtxReadDxl);
    return false;
  }
  result =
      dxl_wb.getRawBulkReadData(&bulkReadID[0], readCount, &bulkReadAddress[0],
                                &bulkReadLength[0], &bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("dxlBulkRead error3: %s", log);
    pthread_mutex_unlock(&mtxReadDxl);
    return false;
  }

  pthread_mutex_unlock(&mtxReadDxl);
  return true;
}

bool SensorBulkWrite(uint8_t WriteCount, uint8_t *bulkWriteID,
                     uint16_t *bulkWriteAddress, uint16_t *bulkWriteLenght,
                     int32_t *bulkWriteData) {
  const char *log = NULL;
  bool result = false;

  for (uint8_t index = 0; index < WriteCount; index++) {
    result = dxl_wb.addBulkWriteParam(
        bulkWriteID[index], bulkWriteAddress[index], bulkWriteLenght[index],
        bulkWriteData[index], &log);
    if (result == false) {
      ROS_ERROR("addBulkWriteParam() %s", log);
      return false;
    }
  }
  result = dxl_wb.bulkWrite(&log);
  if (result == false) {
    ROS_ERROR("bulkWrite() %s\n", log);
    return false;
  }
  return true;
}

// bool ServoMovingGet() {
//   /* at least 1 servo is moving return true ,else return false */
//   int32_t movingGet[20] = {0};

//   ServoBulkRead(dxlIds, numberOfId, "Moving", &movingGet[0]);
//   for (uint8_t idNum = 0; idNum < numberOfId; idNum++) {
//     if (movingGet[idNum]) {
// #ifdef DEBUG
//       ROS_INFO("Moving !!");
// #endif
//       return true;
//     }
//   }
//   return false;
// }

std::vector<double> linspace(double start, double end, int num) {
  std::vector<double> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num);

  for (int i = 0; i <= num; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}

void RobotStand(uint8_t *ids, uint8_t idCnt, int32_t velocity) {
  
  std::vector<double> diffAngle = {0.0, -1.5, 16.0, -32.0, -16.0, -1.5, 0.0, 1.5, -16.0, 32.0,
                                 16.0, 1.5, 0.0, -70.0, -15.0, 0.0, 70.0, 15.0, 0.0, 0.0, 0.0, 0.0};                               
  bodyhub::ServoPositionAngle servoPositionsMsg;

  if (!SimControll::simEnable) {
    const char *log = NULL;
    uint8_t idArray[30] = {0};
    int32_t standPositionGoal[30];
    double_t linspaceTime;
    uint8_t intervalCount;
    std::vector<std::vector<double>> motoMoveSequence(30);
    std::cout << "diffAngle:" << std::endl;
    for (uint8_t i = 0; i < SERVO_NUM; i++) {
      diffAngle[i] = fabs(diffAngle[i] - ServoStore.angle[i]);
      printf("%.1f  ",diffAngle[i]);
    }
    std::cout << std::endl;
    auto maxAngle = max_element(diffAngle.begin(), diffAngle.end());   
    linspaceTime = *maxAngle / 100;    //  100°/s
    intervalCount = linspaceTime / mWalk.timeStep;  //插值次数
    printf("maxAngle：%.2f, linspaceTime: %.2f, intervalCount: %d \n", *maxAngle, linspaceTime, intervalCount);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) { 
      idArray[idNum] = ids[idNum];
      standPositionGoal[idNum] =
          convertAngleTOValue(idArray[idNum], standPos[idArray[idNum] - 1]);
      motoMoveSequence[idNum] =
          linspace(ServoStore.value[idArray[idNum] - 1],
                   standPositionGoal[idNum], intervalCount);
      ServoStore.angle[idArray[idNum] - 1] = standPos[idArray[idNum] - 1];
      ServoStore.value[idArray[idNum] - 1] = standPositionGoal[idNum];
    }
    ros::Rate loopRate(1.0 / mWalk.timeStep);
    for (uint8_t standCycleT = 0; standCycleT < intervalCount; standCycleT++) {
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        standPositionGoal[idNum] = motoMoveSequence[idNum][standCycleT] +
                                   servoOffset[idArray[idNum] - 1];
      dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray, idCnt,
                       standPositionGoal, 1, &log);  //同步写指令             
                                                     // TODO: fail return                                                                                                                                          
      loopRate.sleep();
    }
  } else  // simulation joint command update
  {
    int16_t frameNumber = 1.5 / mWalk.timeStep;  // 1.5s
    std::vector<std::vector<double>> positionVector;
    std::vector<double> jointPosition;
    std_msgs::Float64MultiArray getPosition =
        SimControll::SimRobotData.getJointPostion();  // radian

    for (uint8_t i = 0; i < SERVO_NUM; i++) {
      positionVector.push_back(linspace(
          getPosition.data[i], standPos[i] * Util::TO_RADIAN, frameNumber));
      ServoStore.angle[i] = standPos[i];
    }
    for (uint16_t n = 0; n < frameNumber; n++) {
      for (uint16_t m = 0; m < SERVO_NUM; m++) {
        if ((m >= 12) && (armMode == 0))
          jointPosition.push_back(getPosition.data[m]);
        else
          jointPosition.push_back(positionVector[m][n]);
      }
      SimControll::updateJointCmdQueue(jointPosition);  // radian
      jointPosition.clear();
    }
  }
  servoPositionsMsg.angle = ServoStore.angle;
  ServoPositionPub.publish(servoPositionsMsg);
}

void motionToPosture(double_t duration, uint8_t number, uint8_t *ids,
                     double_t *position) {
  bool result;
  const char *log = NULL;
  int32_t readData = 0;
  uint32_t numberOfFrame = duration / mWalk.timeStep;  // duration s
  std::vector<std::vector<double>> positionVector;
  int32_t motorPos[number];

  for (uint8_t i = 0; i < number; i++) {
    for (uint8_t c = 0; c < 3; c++)  // read 3 times
    {
      result = dxl_wb.readRegister(ids[i], "Present_Position", &readData, &log);
      if (result == true) break;
    }
    if (result == false) {
      ROS_ERROR("motionToPosture error: ID is %d, %s", ids[i], log);
    } else {
      ServoStore.angle[ids[i] - 1] =
          convertValueTOAngle(ids[i], readData - servoOffset[ids[i] - 1]);
      ServoStore.value[ids[i] - 1] = readData;
    }
  }
  // for (uint8_t i = 0; i < SERVO_NUM; i++)
  //   ROS_INFO("ServoStore.angle[%d]: %f", i, ServoStore.angle[i]);

  double_t targetValue;
  for (uint8_t i = 0; i < number; i++) {
    targetValue =
        convertAngleTOValue(ids[i], position[i]) + servoOffset[ids[i] - 1];
    positionVector.push_back(
        linspace(ServoStore.value[ids[i] - 1], targetValue, numberOfFrame));
    ServoStore.angle[ids[i]] = position[i];
    ServoStore.value[ids[i] - 1] = targetValue;
  }
  ros::Rate loopRate(1.0 / mWalk.timeStep);
  for (uint16_t n = 0; n < numberOfFrame; n++) {
    for (uint16_t m = 0; m < number; m++) {
      motorPos[m] = positionVector[m][n];
    }
    result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, ids, number,
                              motorPos, 1, &log);  //同步写指令                            
    if (result == false) {
      ROS_ERROR("motionToPosture error: %s", log);
    }
    loopRate.sleep();
  }
}

void LoadOffset(const std::string path) {
  std::map<std::string, double> offsetMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node offsetDoc;
  try {
    offsetDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node itemData = offsetDoc["offset"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double offsetValue = itItemNum->second.as<double>();

    offsetMap[IDName] = offsetValue;
  }
  std::cout << "servo offset:\n";
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    IDNameStr = stringID + std::to_string(i + 1);
    if (offsetMap.find(IDNameStr) == offsetMap.end())
      ROS_WARN("\nwithout find offset of %s ", IDNameStr.c_str());
    else {
      servoOffset[i] = offsetMap[IDNameStr];
    }
    std::cout << servoOffset[i] << "  ";
  }
  std::cout << "\n";
}

void LoadDxlInitPose(const std::string path) {
  std::map<std::string, double> initPosMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node initPoseDoc;
  try {
    initPoseDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load dxlinitPose yaml.");
    return;
  }

  YAML::Node itemData = initPoseDoc["InitPose"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double initPoseValue = itItemNum->second.as<double>();

    initPosMap[IDName] = initPoseValue;
  }
  std::cout << "stand position:\n";
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    IDNameStr = stringID + std::to_string(i + 1);
    if (initPosMap.find(IDNameStr) == initPosMap.end())
      ROS_WARN("\nwithout find standPos of %s ", IDNameStr.c_str());
    else {
      standPos[i] = initPosMap[IDNameStr];
    }
    std::cout << standPos[i] << "  ";
  }
  std::cout << "\n";
}

void jointControlCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  if (!mtxFSM.try_lock()) return;
  uint16_t msgId;
  try {
    msgId = std::stoi(msg->name.at(msg->name.size() - 1));
  } catch (const std::exception &e) {
    std::cerr << "catch error: " << e.what() << '\n';
    return;
  }

  if (currentControlId == msgId) {
    sensor_msgs::JointState jointStateMsg;
    jointStateMsg = *msg;
    jointStateMsg.name.pop_back();
    for (uint8_t i = 0; i < jointStateMsg.name.size(); i++) {
      if (i >= jointStateMsg.position.size())
        jointStateMsg.position.push_back(0.0f);

      if (i >= jointStateMsg.velocity.size())
        jointStateMsg.velocity.push_back(0.0f);

      if (i >= jointStateMsg.effort.size())
        jointStateMsg.effort.push_back(0.0f);
    }

    mtxJointTrajQueue.lock();
    jointTrajQueue.push(jointStateMsg);
    mtxJointTrajQueue.unlock();
    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  mtxFSM.unlock();
}

void MotoPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  if (!mtxFSM.try_lock()) return;
  if (currentControlId == msg->mainControlID) {
    bodyhub::JointControlPoint jointControlMsg;
    jointControlMsg.mainControlID = msg->mainControlID;
    jointControlMsg.positions = msg->positions;
    for (uint8_t i = 0; i < msg->positions.size(); i++) {
      if (i < msg->velocities.size())
        jointControlMsg.velocities.push_back(msg->velocities.at(i));
      else
        jointControlMsg.velocities.push_back(0.0f);

      if (i < msg->accelerations.size())
        jointControlMsg.accelerations.push_back(msg->accelerations.at(i));
      else
        jointControlMsg.accelerations.push_back(0.0f);
    }

    pthread_mutex_lock(&mtxMo);
    motoQueue.push(jointControlMsg);
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  mtxFSM.unlock();
}

void HeadPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  /* 仅头部舵机数据接收 0_ID19 1_ID20 */
  if (!mtxFSM.try_lock()) return;
  if (currentControlId == msg->mainControlID) {
    bodyhub::JointControlPoint jointControlMsg;
    jointControlMsg.mainControlID = msg->mainControlID;
    jointControlMsg.positions = msg->positions;
    for (uint8_t i = 0; i < msg->positions.size(); i++) {
      if (i < msg->velocities.size())
        jointControlMsg.velocities.push_back(msg->velocities.at(i));
      else
        jointControlMsg.velocities.push_back(0.0f);

      if (i < msg->accelerations.size())
        jointControlMsg.accelerations.push_back(msg->accelerations.at(i));
      else
        jointControlMsg.accelerations.push_back(0.0f);
    }

    pthread_mutex_lock(&mtxMo);
    headCtrlQueue.push(jointControlMsg);
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  mtxFSM.unlock();
}

bool InstReadValSrvCallback(bodyhub::SrvInstRead::Request &req,
                            bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    result = dxl_wb.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = SERVO_DEFAULT_VALUE;
    } else
      res.getData = readGetData;
  } else {
    res.getData = SERVO_DEFAULT_VALUE;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool InstWriteValSrvCallback(bodyhub::SrvInstWrite::Request &req,
                             bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    writeSetData = req.setData;
    dxl_wb.writeOnlyRegister(dxlID, itemName.c_str(), writeSetData, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SyncWriteValSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                             bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    result = dxl_wb.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令                            
  } else
    ROS_WARN("YOU ARE NOT IN directOperate");

  res.complete = result;
  return true;
}

bool SetServoTarPositionValCallback(bodyhub::SrvServoWrite::Request &req,
                                    bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      jointPosition.data[req.dxlID] =
          Angle2Radian(convertValueTOAngle(req.dxlID, req.setData));
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      dxlID = req.dxlID;
      writeSetData = req.setData;
      dxl_wb.writeOnlyRegister(dxlID, "Goal_Position", writeSetData, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool SetServoTarPositionValAllCallback(
    bodyhub::SrvServoAllWrite::Request &req,
    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        jointPosition.data[req.idArray[i] - 1] =
            Angle2Radian(convertValueTOAngle(req.idArray[i], req.setData[i]));
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }   
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, writeSetData, 1, &log);  //同步写指令                                                                                                          
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool GetServoPositionValAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                    bodyhub::SrvServoAllRead::Response &res) {
  int32_t defaultValue = 2048;
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[30] = {
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue, defaultValue, defaultValue, defaultValue,
      defaultValue, defaultValue};

  //读值前使用上次舵机下发值
  for (uint8_t i = 0; i < numberOfId; i++)
    readGetData[i] = ServoStore.value[i];
   
  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        res.getData.push_back(convertAngleTOValue(
            req.idArray[i],
            Radian2Angle(jointPosition.data[req.idArray[i] - 1])));
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
      }
      ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        res.getData.push_back(readGetData[idNum]);  

      if (numberOfId != 22) {
        ROS_WARN("Failed to scan %d servo(s)",SERVO_NUM - numberOfId);
        for (uint8_t i = 0; i < SERVO_NUM - numberOfId; i++) {
        if (failed_ID[i] == 0)
          break;
        res.getData.insert(res.getData.begin()+failed_ID[i]-1, ServoStore.value[failed_ID[i]-1]);
        res.getData.pop_back();
        ROS_INFO("Insert ServoStore value in ID: %d, delete error value. times: %d",
                  failed_ID[i], i+1);
        }
      }                     
    }
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool InstReadSrvCallback(bodyhub::SrvInstRead::Request &req,
                         bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    result = dxl_wb.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = 998;
    } else if (itemName == "Present_Position")
      res.getData = convertValueTOAngle(dxlID, readGetData);  // value_to_angle
    else
      res.getData = readGetData;
  } else {
    res.getData = 999;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool InstWriteSrvCallback(bodyhub::SrvInstWrite::Request &req,
                          bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    if (itemName == "Goal_Position")
      writeSetData = convertAngleTOValue(dxlID, req.setData);
    else
      writeSetData = req.setData;
    dxl_wb.writeOnlyRegister(dxlID, itemName.c_str(), writeSetData, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool SyncWriteSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                          bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] =
            convertAngleTOValue(idArray[idNum], req.setData[idNum]);
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    result = dxl_wb.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令                             
  } else
    ROS_WARN("YOU ARE NOT IN directOperate");

  res.complete = result;
  return true;
}

bool SetServoLockStateCallback(bodyhub::SrvServoWrite::Request &req,
                               bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    writeSetData = (int32_t)req.setData;
    if (writeSetData == 0)
      dxl_wb.torqueOff(dxlID, &log);
    else
      dxl_wb.torqueOn(dxlID, &log);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoLockStateAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                  bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];       // test
      writeSetData[idNum] = req.setData[idNum];  // test
      if (writeSetData[idNum] == 0)
        dxl_wb.writeRegister(idArray[idNum], "Torque_Enable", 0, &log);
      else if (writeSetData[idNum] == 1)
        dxl_wb.writeRegister(idArray[idNum], "Torque_Enable", 1, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool GetServoLockStateAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                  bodyhub::SrvServoAllRead::Response &res) {
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[30] = {0};

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];
    }
    ServoBulkRead(idArray, idCnt, "Torque_Enable", &readGetData[0]);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++)
      res.getData.push_back(readGetData[idNum]);
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoTarPositionCallback(bodyhub::SrvServoWrite::Request &req,
                                 bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      jointPosition.data[req.dxlID] = Angle2Radian(req.setData);
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      dxlID = req.dxlID;
      writeSetData = convertAngleTOValue(dxlID, req.setData);
      dxl_wb.writeRegister(dxlID, "Goal_Position", writeSetData, &log);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool SetServoTarPositionAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        jointPosition.data[req.idArray[i] - 1] = Angle2Radian(req.setData[i]);
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] =
            convertAngleTOValue(req.idArray[idNum], req.setData[idNum]);
      }
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, writeSetData, 1, &log);  //同步写指令                            
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}
bool GetServoPositionAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                 bodyhub::SrvServoAllRead::Response &res) {
  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        res.getData.push_back(
            Radian2Angle(jointPosition.data[req.idArray[i] - 1]));
    } else {
      uint8_t idArray[30] = {0};
      uint8_t idCnt = 0;
      int32_t readGetData[30] = {0};

      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
      }
      ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        res.getData.push_back(convertValueTOAngle(
            req.idArray[idNum], readGetData[idNum]));  // value_to_angle
    }
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
  }

  return true;
}

bool StateSrvCallback(bodyhub::SrvState::Request &req,
                      bodyhub::SrvState::Response &res) {
  if (mtxFSM.try_lock()) {
    if ((req.masterID == currentControlId) || (currentControlId == 0)) {
      if (req.stateReq == "setStatus") {
        currentControlId = req.masterID;
        if (bodyhubState == StateEnum::preReady) UpdateState(StateEnum::ready);
      } else if (req.stateReq == "resetStatus") {
      } else if (req.stateReq == "break") {
      } else if (req.stateReq == "stop") {
        if ((bodyhubState == StateEnum::running) ||
            (bodyhubState == StateEnum::pause) ||
            (bodyhubState == StateEnum::walking)) {
          pthread_mutex_lock(&mtxWl);
          ClearTimerQueue();
          UpdateState(StateEnum::stoping);
          UpdateState(StateEnum::ready);
          pthread_mutex_unlock(&mtxWl);
        }
      } else if (req.stateReq == "reset") {
        if (bodyhubState != StateEnum::stoping) {
          UpdateState(StateEnum::preReady);
          currentControlId = 0;
        }
      } else if (req.stateReq == "walking") {
        if ((bodyhubState == StateEnum::ready) ||
            (bodyhubState == StateEnum::pause)) {
          UpdateState(StateEnum::walking);
        }
      }
      res.stateRes = bodyhubState;
    } else
      res.stateRes = currentControlId;
    mtxFSM.unlock();
    return true;
  }
  ROS_WARN("FSM service busy!");
  return false;
}

bool GetStatusCallback(bodyhub::SrvString::Request &req,
                       bodyhub::SrvString::Response &res) {
  if (req.str != "") {
    res.data = stateNewStr;
    res.poseQueueSize = motoQueue.size();
    res.jointQueueSize = jointTrajQueue.size();
  }
  return true;
}

bool GetJointAngleCallback(bodyhub::SrvServoAllRead::Request &req,
                           bodyhub::SrvServoAllRead::Response &res) {
  res.getData = ServoStore.angle;
  return true;
}

bool MasterIDSrvCallback(bodyhub::SrvTLSstring::Request &req,
                         bodyhub::SrvTLSstringResponse &res) {
#ifdef DEBUG
// ROS_INFO("MasterIDSrvCallback get masterID %s", req.str.c_str());
#endif
  res.data = currentControlId;
  return true;
}

void *queueThread(void *ptr) {
  ROS_INFO("Start 'queueThread' thrand...");
  ros::NodeHandle n;
  ros::CallbackQueue topicQueue;

  n.setCallbackQueue(&topicQueue);

  ros::Subscriber MotoPositionSub = n.subscribe(
      "MediumSize/BodyHub/MotoPosition", 1000, MotoPositionCallback);
  ros::Subscriber HeadPositionSub = n.subscribe(
      "MediumSize/BodyHub/HeadPosition", 1000, HeadPositionCallback);

  ros::Subscriber jointControlSub = n.subscribe(
      "MediumSize/BodyHub/jointControl", 1000, jointControlCallback);

  while (n.ok()) {
    topicQueue.callAvailable(ros::WallDuration(0.01));
  }
  return NULL;
}

bool initDxlHandlers(void) {
  bool result = false;
  const char *log = NULL;

  result = dxl_wb.addSyncWriteHandler(dxlIds[0], "Goal_Position", &log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxl_wb.addSyncWriteHandler(dxlIds[0], "Moving_Speed",
                                      &log);  //第一个舵机号要为存在舵机
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxl_wb.initBulkRead(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxl_wb.initBulkWrite(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    return result;
  }

  return result;
}

void ReadJointValue() {
  uint8_t failed_cnt = 0;
  const char *log = NULL;
  bool result = false;
  int32_t readGetData = 0;
  for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++) {
    for (uint8_t i = 0; i < 3; i++) {
      result = dxl_wb.readRegister(idNum + 1, "Present_Position", &readGetData,
                                   &log);
      if (result == true) break;
    }
    if (result == false) {
      ROS_ERROR("failed to read servo, ID is %d, %s", idNum + 1, log);
      failed_ID[failed_cnt] = idNum + 1;
      failed_cnt++;
    } else {
      ServoStore.value[idNum] = readGetData - servoOffset[idNum];
      ServoStore.angle[idNum] =
          convertValueTOAngle(idNum + 1, ServoStore.value[idNum]);
    }
  }
  failed_cnt = 0;
}

void UpdateState(uint8_t stateNew) {
  bodyhub::ServoPositionAngle servoPositionsMsg;
  const char *log = NULL;
  bool result = false;

  if ((stateNew == StateEnum::preReady) || (stateNew == StateEnum::stoping)) {
    if (!SimControll::simEnable) {
      mWalk.quitWalk();
      ReadJointValue();
      servoPositionsMsg.angle = ServoStore.angle;
      ServoPositionPub.publish(servoPositionsMsg);
      if (bodyhubState == StateEnum::directOperate) {
        // reload offset
        if (offsetFile != "") LoadOffset(offsetFile);
#ifdef DEBUG
        for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++)
          ROS_INFO("directOperate exit Servo Present_Position ID%d: %f", idNum,
                   ServoStore.value[idNum]);
#endif
      }
      result = dxl_wb.writeOnlyRegister(254, "P_gain", 6, &log);  // init
    } else {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < jointPosition.data.size(); i++)
        servoPositionsMsg.angle.push_back(Radian2Angle(jointPosition.data[i]));
      ServoPositionPub.publish(servoPositionsMsg);
    }
    if (!noRobotStand) RobotStand(dxlIds, numberOfId, 1000);
    ClearTimerQueue();
  } else if (stateNew == StateEnum::walking) {
    if (!SimControll::simEnable) {
      // P P P P P P_gain
      int16_t P_gainList[22] = {
          20, 35, 30, 30, 30, 35, 20, 35, 30, 30, 30, 35,
      };
      for (uint16_t i = 0; i < 12; i++) {
        result = dxl_wb.writeOnlyRegister(i + 1, "P_gain", P_gainList[i], &log);
        if (result == false) {
          printf("Failed to P_gain: %d, %s\n", i + 1, log);
        }
      }
      // hand
      result = dxl_wb.writeOnlyRegister(13, "P_gain", 8, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxl_wb.writeOnlyRegister(16, "P_gain", 8, &log);
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
    }
    // RobotSquat
    Eigen::VectorXd currentJointV;
    currentJointV.resize(SERVO_NUM);
    std::vector<Eigen::VectorXd> squatSequence;
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();

      for (uint8_t i = 0; i < jointPosition.data.size(); i++) {
        currentJointV[i] =
            WalkJointDirection[i] * Radian2Angle(jointPosition.data.at(i));
      }
    } else {
      ReadJointValue();
      for (uint16_t i = 0; i < SERVO_NUM; i++) {
        currentJointV[i] = ServoStore.angle[i] * WalkJointDirection[i];
        // std::cout<<"currentJointV["<<i<<"]: "<<currentJointV[i]<<"\n";
      }
    }
    ros::Rate loopRate(1.0 / mWalk.timeStep);
    squatSequence = mWalk.initSquat(currentJointV);
    if(!isJointError())
    {
      mWalk.squatCount = 0;
      for (uint8_t cycleCount = 0; cycleCount < squatSequence.size();
          cycleCount++) {
        mWalk.squatCount++;
        mWalk.jointValue.segment(0, 12) =
            squatSequence[cycleCount];  // FIXME: (0,22)
        WalkingSendData();
        loopRate.sleep();
      }
    }
  } else if (stateNew == StateEnum::ready) {
    if (SimControll::simEnable && SimControll::simState != 1)
      SimControll::simStart();
    else
      servoParamInit(dxlIds, numberOfId);
  } else if (stateNew == StateEnum::error) {
    if (SimControll::simEnable) SimControll::simPause();
  }

  bodyhubState = stateNew;
  budyhubStateMsg.data = stateNew;
  StatusPub.publish(budyhubStateMsg);

  switch (stateNew) {
    case StateEnum::init:
      stateNewStr = "init";
      break;
    case StateEnum::preReady:
      stateNewStr = "preReady";
      break;
    case StateEnum::ready:
      stateNewStr = "ready";
      break;
    case StateEnum::running:
      stateNewStr = "running";
      break;
    case StateEnum::pause:
      stateNewStr = "pause";
      break;
    case StateEnum::stoping:
      stateNewStr = "stoping";
      break;
    case StateEnum::error:
      stateNewStr = "error";
      break;
    case StateEnum::directOperate:
      stateNewStr = "directOperate";
      break;
    case StateEnum::walking:
      stateNewStr = "walking";
      break;
    default:
      break;
  }
  ROS_INFO("The new bodyhubState: %s--%d ", stateNewStr.c_str(), stateNew);
}

void ClearTimerQueue() {
  pthread_mutex_lock(&mtxMo);
  ClearQueue(motoQueue);
  pthread_mutex_unlock(&mtxMo);

  pthread_mutex_lock(&mtxHe);
  ClearQueue(headCtrlQueue);
  pthread_mutex_unlock(&mtxHe);

  if (!jointTrajQueue.empty()) {
    std::queue<sensor_msgs::JointState> emptyQ;
    mtxJointTrajQueue.lock();
    jointTrajQueue.swap(emptyQ);
    mtxJointTrajQueue.unlock();
  }
}

bool dxlReadImu(double_t *gyro, double_t *acc) {
  bool result = false;
  const char *log = NULL;
  uint32_t getData[BASE_BOARD_LEN];

  result = dxl_wb.readRegister(BASE_BOARD_ID, BASE_BOARD_ADDR, BASE_BOARD_LEN,
                               getData, &log);
  if (result == false) {
    ROS_ERROR("read imu error, %s", log);
    return false;
  }

  uint8_t startAddr = BASE_BOARD_ADDR;
  gyro[0] = GYRO_COEFFICIENT *
            TO_INT16(getData[38 - startAddr], getData[39 - startAddr]);
  gyro[1] = GYRO_COEFFICIENT *
            TO_INT16(getData[40 - startAddr], getData[41 - startAddr]);
  gyro[2] = GYRO_COEFFICIENT *
            TO_INT16(getData[42 - startAddr], getData[43 - startAddr]);
  acc[0] = ACC_COEFFICIENT *
           TO_INT16(getData[44 - startAddr], getData[45 - startAddr]);
  acc[1] = ACC_COEFFICIENT *
           TO_INT16(getData[46 - startAddr], getData[47 - startAddr]);
  acc[2] = ACC_COEFFICIENT *
           TO_INT16(getData[48 - startAddr], getData[49 - startAddr]);

  return true;
}

void imuCheck() {
  uint16_t count = 0;
  double gyro[3], acc[3];
  ros::Rate loopRate(100);

  imuType = 1;
  count = 10;
  while (ros::ok() && count) {
    count--;
    if (dxlReadImu(gyro, acc) == true) {
      if (((gyro[0] != 0.0) && (gyro[1] != 0.0) && (gyro[2] != 0.0)) &&
          ((gyro[0] != 0.0) && (gyro[1] != 0.0) && (gyro[2] != 0.0))) {
        imuType = 0;
      }
    }
    loopRate.sleep();
  }
}

void imuInit() {
  uint16_t number = 20;
  uint16_t count = 0;
  double gyro[3], acc[3];
  double gyroSum[3], accSum[3];
  ros::Rate loopRate(100);

  imuCheck();
  if (imuType == 0) {
    ROS_INFO("calibrate imu...");
    while (ros::ok() && count) {
      if (dxlReadImu(gyro, acc) == true) {
        count--;
        for (uint16_t i = 0; i < 3; i++) {
          gyroSum[i] += gyro[i];
          accSum[i] += acc[i];
        }
      }
      loopRate.sleep();
    }
    for (uint16_t i = 0; i < 3; i++) {
      gyro[i] = gyroSum[i] / number;
      acc[i] = accSum[i] / number;
    }
    torsoImu.calibrateGyro(gyro[1], gyro[0], -gyro[2]);
#if 1
    std::cout << "imuOffset gx:" << gyro[1] << std::setw(10) << "gy:" << gyro[0]
              << std::setw(10) << "gz:" << -gyro[2] << std::setw(10) << "\n";
#endif
    ROS_INFO("calibrate imu over.");

    torsoImu.setGyroLpf(100, 20);
    torsoImu.setAccLpf(100, 20);

    count = 100;
    while (ros::ok() && count) {
      if (dxlReadImu(gyro, acc) == true) {
        count--;
        torsoImu.setGyro(gyro[1], gyro[0], -gyro[2]);
        torsoImu.setAcc(acc[1], acc[0], -acc[2]);
        torsoImu.datafusion();
      }
      loopRate.sleep();
    }
  } else {
    extern void jy901ModuleInit(void);
    jy901ModuleInit();
  }
}

#if 0
void servoParamInit() 
{
  bool result = false;
  const char *log = NULL;
  int32_t param = 40;

  result = dxl_wb.addSyncWriteHandler(dxlIds[0], "P_gain", &log);
  if (result == false) {
    ROS_WARN("setParam failed, %s", log);
    return;
  }

  result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, dxlIds, numberOfId,
                            &param, 1, &log);                             
  if (result == false) {
    ROS_WARN("setParam failed, %s", log);
    return;
  }
}
#else
void servoParamInit(uint8_t *ids, uint8_t number) {
  bool result = false;
  const char *log = NULL;
  int32_t pGain[22] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
                       30, 15, 50, 50, 15, 50, 50, 50, 50, 80, 80};
  int32_t maxTorque[22] = {0,    0,    0,    0,    0, 0,    0,    0,
                           0,    0,    0,    0,    0, 1023, 1023, 1023,
                           1023, 1023, 1023, 1023, 0, 0};

  std::cout << "set P_gain\n";
  for (uint8_t i = 0; i < number; i++) {
    result =
        dxl_wb.writeOnlyRegister(ids[i], "P_gain", pGain[ids[i] - 1], &log);
    if (result == false) {
      ROS_WARN("set p_gain %d failed, %s", ids[i], log);
    }
  }
  std::cout << std::endl;

  std::cout << "set Max_Torque miniSERVO\n";
  for (uint8_t i = 0; i < number; i++) {
    if (maxTorque[ids[i] - 1] == 0) continue;
    result = dxl_wb.writeOnlyRegister(ids[i], "Max_Torque",
                                      maxTorque[ids[i] - 1], &log);
    if (result == false) {
      ROS_WARN("set Max_Torque %d failed, %s", ids[i], log);
    }
  }
  std::cout << std::endl;

  std::cout << "set torque\n";
  for (uint8_t i = 0; i < number; i++) {
    result = dxl_wb.writeOnlyRegister(ids[i], "Torque_Enable", 1, &log);
    if (result == false) {
      ROS_WARN("torqueOn %d failed, %s", ids[i], log);
    }
  }
  std::cout << std::endl;
}
#endif

static uint8_t portInit(const char *portName, uint32_t baudrate) {
  const char *log = NULL;
  bool result = false;
  result = dxl_wb.init(portName, baudrate, &log);  // initWorkbench
  if (result == false) {
    return 1;
  }
  for (uint8_t s_num = 0; s_num <= 2; s_num++) {
    dxl_wb.scan(dxlIds, &numberOfId, DXL_ID_SCAN_RANGE, &log);
    if (numberOfId == SERVO_NUM) {
      break;
    }
    usleep(1000 * 20);
  }
  if (numberOfId == 0) {
    return 2;
  }

  return 0;
}

bool servoInit() {
  const char *log = NULL;
  bool result = false;
  uint8_t ret = 0;
  const char *portName = "/dev/ttyUSB0";
  uint32_t baudrate[] = {1000000, 2000000};
  for (uint16_t i = 0; i < 2; i++) {
    std::cout << "try baudrate: " << baudrate[i] << "\n";
    ret = portInit(portName, baudrate[i]);
    if (ret == 0) break;
  }
  if (ret != 0) {
    switch (ret) {
      case 1:
        ROS_ERROR("open port failed!");
        break;
      case 2:
        ROS_ERROR("no dynamixel device found!");
        break;
    }
    return false;
  }

  std::cout << "device id:" << std::endl;
  for (uint8_t i = 0; i < numberOfId; i++) {
    std::cout << (int)dxlIds[i] << "  ";
  }
  std::cout << std::endl;

  result = initDxlHandlers();
  if (result == false) {
    ROS_ERROR("initDxlHandlers failed!");
    return false;
  }
  servoParamInit(dxlIds, numberOfId);
  return true;
}

void sigintHandler(int sig) {
#if 0  //
  if(!SimControll::simEnable)
  {
    uint8_t ids[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
    double_t position[12] = {0, 0, 65, -105, -42, 0, 0, 0, -65, 105, 42, 0};
    motionToPosture(1.5, 12, ids, position);
  }
#endif

  if (SimControll::simEnable) SimControll::simStop();
  ros::shutdown();
}

void LoadAssembleParam(ros::NodeHandle nh) {
  nh.param<std::string>("poseOffsetPath", offsetFile, "");
  if (access(offsetFile.c_str(), F_OK) != 0) {
    nh.param<std::string>("packOffsetFile", offsetFile, "");
  }
}

bool imuStateCallback(bodyhub::SrvimuState::Request &req,
                      bodyhub::SrvimuState::Response &res) {
  ROS_INFO("Request from imuState : %s",req.state.c_str());
  if (req.state == "on") {
    setRestrainWobbleOnoff(true);
    res.freeback = "[server]Successful!Now imuState is on";
  } else if (req.state == "off") {
    setRestrainWobbleOnoff(false);
    res.freeback = "[server]Successful!Now imuState is off";
  } else {
    res.freeback = "[server]Failed!Please input on or off";
  }  

  return true;
}

void controllerParamInit() {
  double_t wobble_k[] = {0.035, 0.03};
  setRestrainWobbleParam(wobble_k);
  setRestrainWobbleOnoff(true);
  double_t balanceRoll_k[] = {1.2, 0.04, 0.1, 0.04};
  double_t balancePitch_k[] = {1.1, 0.02, 0.1, 0.04};
  setSlopeBalanceRParam(balanceRoll_k);
  setSlopeBalancePParam(balancePitch_k);
  setSlopeBalanceOnoff(true);
}

void STATEinit(ros::NodeHandle nh) {
  

  StatusPub = nh.advertise<std_msgs::UInt16>("MediumSize/BodyHub/Status", 0);
  ServoPositionPub = nh.advertise<bodyhub::ServoPositionAngle>(
      "MediumSize/BodyHub/ServoPositions", 0, true);
  imuStateService =
      nh.advertiseService("imuState", imuStateCallback);
  StateService =
      nh.advertiseService("MediumSize/BodyHub/StateJump", StateSrvCallback);
  GetStatusService =
      nh.advertiseService("MediumSize/BodyHub/GetStatus", GetStatusCallback);
  MasterIDService = nh.advertiseService("MediumSize/BodyHub/GetMasterID",
                                        MasterIDSrvCallback);
  GetJointAngleService = nh.advertiseService("MediumSize/BodyHub/GetJointAngle",
                                             GetJointAngleCallback);

  // VALUE
  InstReadValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstReadVal", InstReadValSrvCallback);
  InstWriteValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWriteVal", InstWriteValSrvCallback);
  SyncWriteValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWriteVal", SyncWriteValSrvCallback);

  SetTarPositionValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionVal",
      SetServoTarPositionValCallback);
  SetTarPositionValAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",
      SetServoTarPositionValAllCallback);
  GetPositionValAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoPositionValAll",
      GetServoPositionValAllCallback);

  // ANGLE
  InstReadService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstRead", InstReadSrvCallback);
  InstWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWrite", InstWriteSrvCallback);
  SyncWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWrite", SyncWriteSrvCallback);

  SetLockStateService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoLockState",
                          SetServoLockStateCallback);
  SetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoLockStateAll",
      SetServoLockStateAllCallback);
  GetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoLockStateAll",
      GetServoLockStateAllCallback);

  SetTarPositionService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoTarPosition",
                          SetServoTarPositionCallback);
  SetTarPositionAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll",
      SetServoTarPositionAllCallback);
  GetPositionAllService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/GetServoPositionAll",
                          GetServoPositionAllCallback);

  /***walking***walking***walking***/
  jointPosTargetPub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/angle/target", 1000);
  jointPosMeasurePub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/angle/measure", 1000);

  jointVelTargetPub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/velocity/target", 1000);
  jointVelMeasurePub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/velocity/measure", 1000);

  contactState_pub = nh.advertise<std_msgs::Float64>("/contactState", 1000);
  stepPhase_pub = nh.advertise<std_msgs::Float64>("/stepPhase", 1000);

  cpref_pub = nh.advertise<std_msgs::Float64>("cpref", 1000);
  cpC_pub = nh.advertise<std_msgs::Float64>("cpC", 1000);

  copm_pub = nh.advertise<std_msgs::Float64>("copm", 1000);
  copD_pub = nh.advertise<std_msgs::Float64>("copD", 1000);
  copref_pub = nh.advertise<std_msgs::Float64>("copref", 1000);

  comRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/com/reference", 1000);
  comMea_pub = nh.advertise<std_msgs::Float64MultiArray>("/com/measure", 1000);
  comEsti_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/com/estimate", 1000);

  comVRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/comV/reference", 1000);
  comVMea_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/comV/measure", 1000);
  comVEsti_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/comV/estimate", 1000);

  leftFootRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/leftFootPR/reference", 1000);
  leftFootMea_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/leftFootPR/measure", 1000);

  rightFootRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/rightFootPR/reference", 1000);
  rightFootMea_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/rightFootPR/measure", 1000);

  Torso_Rpub = nh.advertise<std_msgs::Float64>("Torso_R", 1000);
  Torso_Ppub = nh.advertise<std_msgs::Float64>("Torso_P", 1000);

  imuDxl_pub = nh.advertise<std_msgs::Float64MultiArray>("/imu/dxl", 1000);
  imuTorso_pub = nh.advertise<std_msgs::Float64MultiArray>("/imu/Torso", 1000);


  WalkingStatusPub = nh.advertise<std_msgs::Float64>(
      "/MediumSize/BodyHub/WalkingStatus", 1000);
  /***walking***walking***walking***/

  UpdateState(StateEnum::init);

  // load offset (value)
  if (offsetFile != "") LoadOffset(offsetFile);
  // load dxlinitpose (angle)
  if (InitPoseFile != "") LoadDxlInitPose(InitPoseFile);

  pthread_mutex_init(&mtxWl, NULL);
  pthread_mutex_init(&mtxMo, NULL);
  pthread_mutex_init(&mtxHe, NULL);
  pthread_mutex_init(&mtxSL, NULL);
  pthread_mutex_init(&mtxReadDxl, NULL);

  ServoStore.angle.resize(SERVO_NUM);
  ServoStore.value.resize(SERVO_NUM);
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ServoStore.angle[i] = SERVO_DEFAULT_ANGLE;
    ServoStore.value[i] = SERVO_DEFAULT_VALUE;
  }

  if (SimControll::simEnable)  // Vrep Simulation
  {
    SimControll::simInit(nh);
  } else {
    if (servoInit() == false) {
#if 1
      ROS_ERROR("servoInit failed, press enter to exit!");
      getchar();
      exit(0);
#else
      SimControll::simEnable = true;
      SimControll::simInit(nh);
#endif
    }
    imuInit();
    controllerParamInit();
  }
}

void STATEpreReady() { const char *log = NULL; }

void STATEready() { const char *log = NULL; }
void STATErunning() {
  // check empty & Moving
  if ((motoQueue.empty()) && (headCtrlQueue.empty()) && jointTrajQueue.empty())
    UpdateState(StateEnum::pause);  //更新下一个状态
}
void STATEpause() { const char *log = NULL; }
void STATEstoping() {}
void STATEerror() { const char *log = NULL; }

void STATEdirectOperate() { const char *log = NULL; }

void STATEwalking() { const char *log = NULL; }

int main(int argc, char **argv) {
  //初始化节点
  ros::init(argc, argv, "bodyhub_node");
  ros::NodeHandle nodeHandle;
  signal(SIGINT, sigintHandler);

  /* Load ROS Parameter */
  LoadAssembleParam(nodeHandle);
  nodeHandle.getParam("setnostand", noRobotStand);
  nodeHandle.getParam("simenable", SimControll::simEnable);
  nodeHandle.param<std::string>("poseInitPath", InitPoseFile, "");
  nodeHandle.param<std::string>("sensorNameIDPath", sensorNameIDFile, "");

  STATEinit(nodeHandle);
  extern void actionManageInit();
  actionManageInit();

  if (SimControll::simEnable)
    mWalk.setRunMode(1);
  else
    mWalk.setRunMode(0);
  cpWalk = GaitManager::CPWalking5::GetInstance();
  cpWalk->commandRosInit();

  std::thread threadTimer(timerThread, nullptr);
  std::thread threadQueue(queueThread, nullptr);

  std::thread threadSensor(sensorThread);
  std::thread threadSensorTimer(sensorTimerThread);
  std::thread threadSim(SimControll::simThread);

  if (SimControll::simEnable)
    while (ros::ok() && !SimControll::jointState()) usleep(100 * 1000);
  UpdateState(StateEnum::preReady);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (bodyhubState == StateEnum::preReady)
      STATEpreReady();
    else if (bodyhubState == StateEnum::ready)
      STATEready();
    else if (bodyhubState == StateEnum::running)
      STATErunning();
    else if (bodyhubState == StateEnum::pause)
      STATEpause();
    else if (bodyhubState == StateEnum::stoping)
      STATEstoping();
    else if (bodyhubState == StateEnum::error)
      STATEerror();
    else if (bodyhubState == StateEnum::directOperate)
      STATEdirectOperate();
    else if (bodyhubState == StateEnum::walking)
      STATEwalking();

    ros::spinOnce();
    loop_rate.sleep();
  }

  threadTimer.join();
  threadQueue.join();
  threadSensor.join();
  threadSensorTimer.join();
  threadSim.join();

  return 0;
}
