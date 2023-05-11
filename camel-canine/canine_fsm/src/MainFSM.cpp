//
// Created by hs on 22. 10. 5.
//

#include <canine_fsm/MainFSM.hpp>

pthread_t RTThreadControllerHigh;
pthread_t RTThreadControllerLow;
pthread_t RTThreadStateEstimator;

pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;
pthread_t NRTThreadIMU;
pthread_t NRTThreadCANLF;
pthread_t NRTThreadCANRF;
pthread_t NRTThreadCANLB;
pthread_t NRTThreadCANRB;
pthread_t NRTThreadT265;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

CanMotorLF canLF("can12");//can5 -> can12
CanMotorRF canRF("can13");//can6 -> can13
CanMotorLB canLB("can11");
CanMotorRB canRB("can14");//can9 -> can14

RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR) + "canine/canineV4/urdf/canineV4.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);

DataAnalysis SaveStates;
Command userCommand;
raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"canine/canineV4/urdf/canineV4_with_head.urdf");
RobotVisualization userVisual(&world, robot, &server);
StateEstimator robotstate(model);
LowControlMain LowController;
HighControlMain HighController;

const std::string mComPort = "/dev/ttyACM0";
const mscl::Connection mConnection = mscl::Connection::Serial(mComPort);
mscl::InertialNode node(mConnection);
LordImu3DmGx5Ahrs IMUBase(&node);
T265 TrackingCam;

void *RTControllerThreadHigh(void *arg)
{
    std::cout << "entered #High Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(HIGH_CONTROL_dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            HighController.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, High Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void *RTControllerThreadLow(void *arg)
{
    std::cout << "entered #Low Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(LOW_CONTROL_dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            LowController.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Low Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void* RTStateEstimator(void* arg)
{
    std::cout << "entered #rt_state_estimation_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(ESTIMATOR_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        robotstate.StateEstimatorFunction();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "RT Deadline Miss, state estimation thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}


void* NRTCommandThread(void* arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while (true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT * 1e6);
    }
}

void* NRTVisualThread(void* arg)
{
    std::cout << "entered #nrt_command_thread" << std::endl;
    while (true)
    {
        userVisual.VisualFunction();
        usleep(VISUAL_dT * 1e6);
    }
}

void* NRTT265Thread(void* arg) {
    Vec4<double> tempQuaternion;
    double tempEulerAngle[3];
    double roll;
    double Wx;
    double Wz; /// angular velocity of z-axis
    std::cout << "entered #Command_NRTt265Thread" << std::endl;
    ///
    TrackingCam.SetConfig();
    TrackingCam.ParseData();

    while(true)
    {
        tempQuaternion[0] = TrackingCam.GetT265quat().w;
        tempQuaternion[1] = -TrackingCam.GetT265quat().x;
        tempQuaternion[2] = TrackingCam.GetT265quat().z;
        tempQuaternion[3] = TrackingCam.GetT265quat().y;
        TransformQuat2Euler(tempQuaternion, tempEulerAngle);
        roll = tempEulerAngle[0];
        Wz = TrackingCam.GetT265angVel().y;
        Wx = -TrackingCam.GetT265angVel().x;
        /// 여기서 계속 돌다가 t265 에서 입력 스트림이 들어오면 출력 시작.

//        sharedMemory->baseQuartPosition[0] = TrackingCam.GetT265quat().w;
//        sharedMemory->baseQuartPosition[1] = -TrackingCam.GetT265quat().x;
//        sharedMemory->baseQuartPosition[2] = TrackingCam.GetT265quat().z;
//        sharedMemory->baseQuartPosition[3] = TrackingCam.GetT265quat().y;
//        TransformQuat2Euler(sharedMemory->baseQuartPosition, sharedMemory->baseEulerPosition);

        sharedMemory->basePosition[0] = -TrackingCam.GetT265pos().x ;
        sharedMemory->basePosition[1] = TrackingCam.GetT265pos().z ;
//        sharedMemory->basePosition[2] = TrackingCam.GetT265pos().y + 0.08 + 0.1 * sin(roll);
        sharedMemory->basePosition[2] = TrackingCam.GetT265pos().y + 0.08;

//        /// compensate linear velocity, v_x{s} = v_x{b} - l * w_z
        sharedMemory->baseVelocity[0] = -TrackingCam.GetT265vel().x;
        sharedMemory->baseVelocity[1] = TrackingCam.GetT265vel().z;
        sharedMemory->baseVelocity[2] = TrackingCam.GetT265vel().y;

        sharedMemory->testBasePos[0] = -TrackingCam.GetT265pos().x ;
        sharedMemory->testBasePos[1] = TrackingCam.GetT265pos().z ;
//        sharedMemory->testBasePos[2] = TrackingCam.GetT265pos().y + 0.08 + 0.1 * sin(roll);
        sharedMemory->testBasePos[2] = TrackingCam.GetT265pos().y + 0.08;

        /// compensate linear velocity, v_x{s} = v_x{b} - l * w_z
        sharedMemory->testBaseVel[0] = -TrackingCam.GetT265vel().x;
        sharedMemory->testBaseVel[1] = TrackingCam.GetT265vel().z;
        sharedMemory->testBaseVel[2] = TrackingCam.GetT265vel().y;

        usleep(IMU_dT * 1e6);
    }
}

void* NRTImuThread(void* arg)
{
    std::cout << "entered #nrt_IMU_thread" << std::endl;
    IMUBase.GetCurrentConfig(node);
    IMUBase.SetConfig(1000);
    double* baseEulerAngle;
    double* baseAngularVelocity;
    double* baseLinearAcceleration;
    double yawOffset = 0.0;
    node.setSensorToVehicleRotation_eulerAngles({0.0,0.0,0.0});
//    IMUBase.GetCurrentConfig(node);
    bool isFirstRun = true;
    double tempYaw;
    int multiTurn = 0;
    double baseSingleTurnEulerAngle[3];
    Eigen::Quaternion<double> quaternion;
    Eigen::Vector3d eulerEigen;

    struct timespec TIME_1;
    struct timespec TIME_2;


    while (true)
    {
        clock_gettime(CLOCK_REALTIME,&TIME_1);
        IMUBase.ParseData();
        baseAngularVelocity = IMUBase.GetAngularVelocity();
        baseEulerAngle = IMUBase.GetEulerAngle();
        baseLinearAcceleration = IMUBase.GetLinearAcceleration();
        if(isFirstRun)
        {
            yawOffset = -baseEulerAngle[2];
            isFirstRun = false;
        }
        else
        {
            sharedMemory->baseEulerVelocity[0] = baseAngularVelocity[0];
            sharedMemory->baseEulerVelocity[1] = -baseAngularVelocity[1];
            sharedMemory->baseEulerVelocity[2] = -baseAngularVelocity[2];

            baseSingleTurnEulerAngle[0] = baseEulerAngle[0];
            baseSingleTurnEulerAngle[1] = -baseEulerAngle[1];
            baseSingleTurnEulerAngle[2] = -baseEulerAngle[2]-yawOffset;

            if(tempYaw > 3.1 && -baseEulerAngle[2] < 3.1/2.0)
            {
                multiTurn++;
            }
            if(tempYaw < -3.1 && -baseEulerAngle[2] > -3.1/2.0)
            {
                multiTurn--;
            }

            sharedMemory->baseAcceleration[0] = baseLinearAcceleration[0];
            sharedMemory->baseAcceleration[1] = -baseLinearAcceleration[1];
            sharedMemory->baseAcceleration[2] = -baseLinearAcceleration[2];

            eulerEigen << baseSingleTurnEulerAngle[0], baseSingleTurnEulerAngle[1], baseSingleTurnEulerAngle[2];
            quaternion = Eigen::AngleAxisd(eulerEigen[0], Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(eulerEigen[1], Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(eulerEigen[2], Eigen::Vector3d::UnitZ());

            sharedMemory->baseQuartPosition[0] = quaternion.w();
            sharedMemory->baseQuartPosition[1] = quaternion.x();
            sharedMemory->baseQuartPosition[2] = quaternion.y();
            sharedMemory->baseQuartPosition[3] = quaternion.z();

            sharedMemory->baseEulerPosition[0] = baseSingleTurnEulerAngle[0];
            sharedMemory->baseEulerPosition[1] = baseSingleTurnEulerAngle[1];
            sharedMemory->baseEulerPosition[2] = baseSingleTurnEulerAngle[2] + multiTurn * 2.0 * 3.141592;
        }
        tempYaw = -baseEulerAngle[2];
        usleep(IMU_dT * 1e6);
        clock_gettime(CLOCK_REALTIME,&TIME_2);
//        sharedMemory->solvedGRF[0][0] = timediff_us(&TIME_1, &TIME_2);



//        std::cout<<"[IMU thread] imu : "<<baseAngularVelocity[0]<<", "<<-baseAngularVelocity[1]<<", "<<-baseAngularVelocity[2]<<std::endl;
//        std::cout<<"[NRT IMU THREAD] local time : "<<sharedMemory->localTime<<std::endl;
//        std::cout<<"[NRT IMU THREAD] imu : "<<sharedMemory->baseEulerPosition[0]<<", "<<sharedMemory->baseEulerPosition[1]<<", "<<sharedMemory->baseEulerPosition[2]<<std::endl;
//        std::cout<<"[NRT IMU THREAD] imu : "<<sharedMemory->baseAcceleration[0]<<", "<<sharedMemory->baseAcceleration[1]<<", "<<sharedMemory->baseAcceleration[2]<<std::endl;
//        std::cout<< std::endl;
    }
}

void* NRTCANLF(void* arg)
{
    std::cout << "entered #nrt_can_LF_thread" << std::endl;

    while (true)
    {
        canLF.CanFunction();
    }
}

void* NRTCANRF(void* arg)
{
    std::cout << "entered #nrt_can_RF_thread" << std::endl;

    while (true)
    {
        canRF.CanFunction();
    }
}

void* NRTCANLB(void* arg)
{
    std::cout << "entered #nrt_can_LB_thread" << std::endl;

    while (true)
    {
        canLB.CanFunction();
//        usleep(10);
    }
}

void* NRTCANRB(void* arg)
{
    std::cout << "entered #nrt_can_RB_thread" << std::endl;

    while (true)
    {
        canRB.CanFunction();
    }
}

void clearSharedMemory()
{
    sharedMemory->gaitState = STAND;
    sharedMemory->gaitIteration = 0;
    sharedMemory->gaitChangeFlag = false;

    sharedMemory->newCommand = false;
    sharedMemory->canLFStatus = false;
    sharedMemory->canRFStatus = false;
    sharedMemory->canLBStatus = false;
    sharedMemory->canRBStatus = false;
    sharedMemory->motorStatus = false;
    sharedMemory->motorLFState = false;
    sharedMemory->motorRFState = false;
    sharedMemory->motorLBState = false;
    sharedMemory->motorRBState = false;
    sharedMemory->LowControlState = STATE_LOW_CONTROL_STOP;
    sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->canLFState = CAN_NO_ACT;
    sharedMemory->canRFState = CAN_NO_ACT;
    sharedMemory->canLBState = CAN_NO_ACT;
    sharedMemory->canRBState = CAN_NO_ACT;
    sharedMemory->localTime = 0;
    sharedMemory->FSMState = FSM_INITIAL;
    sharedMemory->isNan = false;
    sharedMemory->bIsEndHome = false;

    for (int index = 0; index < MOTOR_NUM; index++)
    {
        sharedMemory->motorErrorStatus[index] = 0;
        sharedMemory->motorTemp[index] = 0;
        sharedMemory->motorPosition[index] = 0;
        sharedMemory->motorVelocity[index] = 0;
        sharedMemory->motorTorque[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
        sharedMemory->motorVoltage[index] = 0;
    }
    for (int index = 0; index < 3; index++)
    {
        sharedMemory->basePosition[index] = 0;
        sharedMemory->baseVelocity[index] = 0;
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
    }

    sharedMemory->baseQuartPosition[0] = 1.0;
    sharedMemory->baseQuartPosition[1] = 0.0;
    sharedMemory->baseQuartPosition[2] = 0.0;
    sharedMemory->baseQuartPosition[3] = 0.0;
}

void StartFSM()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    clearSharedMemory();

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt2 = generate_nrt_thread(NRTThreadVisual, NRTVisualThread, "nrt_thread2", 1, NULL);
    int thread_id_nrt3 = generate_nrt_thread(NRTThreadCANLF, NRTCANLF, "nrt_thread3", 4, NULL);
    int thread_id_nrt4 = generate_nrt_thread(NRTThreadCANRF, NRTCANRF, "nrt_thread4", 4, NULL);
    int thread_id_nrt5 = generate_nrt_thread(NRTThreadCANLB, NRTCANLB, "nrt_thread5", 4, NULL);
    int thread_id_nrt6 = generate_nrt_thread(NRTThreadCANRB, NRTCANRB, "nrt_thread6", 4, NULL);
    int thread_id_nrt7 = generate_nrt_thread(NRTThreadIMU, NRTImuThread, "nrt_thread7", 7, NULL);
    int thread_id_nrt8 = generate_nrt_thread(NRTThreadT265, NRTT265Thread,"nrt_thread8",7,NULL);

    int thread_id_rt1 = generate_rt_thread(RTThreadControllerHigh, RTControllerThreadHigh, "rt_thread1", 5, 99, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadControllerLow, RTControllerThreadLow, "rt_thread2", 6, 99, NULL);
    int thread_id_rt3 = generate_rt_thread(RTThreadStateEstimator, RTStateEstimator, "rt_thread3", 7, 99,NULL);
}