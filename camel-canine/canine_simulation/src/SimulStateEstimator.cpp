//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulStateEstimator.hpp>

extern pSHM sharedMemory;

CanineFilter::Vec3LPF vecLPF(ESTIMATOR_dT,1000);

SimulStateEstimator::SimulStateEstimator(raisim::ArticulatedSystem* robot, RigidBodyDynamics::Model* model)
    : mDt(ESTIMATOR_dT)
    , mGain(Eigen::Matrix<double,18,18>::Identity() * 50)
    , mModel(model)
    , mRobot(robot)
    , mPosition(raisim::VecDyn(19))
    , mVelocity(raisim::VecDyn(18))
    , mGeneralizedForce(Eigen::VectorXd::Zero(18))
    , mbIsFirstFiltering(true)
    , mbIsFirstRun(true)
{
    for(int i=0; i<4; i++)
    {
        mbContactState[i] = true;
        mbIsFirstCon[i] = true;
    }
    mPrevBasePos[0] = 0.003;
    mPrevBasePos[1] = 0.001;
    mPrevBasePos[0] = 0.04413;
    mBeta = Eigen::VectorXd::Zero(18);
    mCalcTorque = Eigen::VectorXd::Zero(18);
    mH = Eigen::MatrixXd::Zero(18,18);
    mMomentum = Eigen::VectorXd::Zero(18);
    mPrevH = Eigen::MatrixXd::Zero(18,18);
    mPrevQ = Eigen::VectorXd::Zero(19);
    mPrevQd = Eigen::VectorXd::Zero(18);
    mQ = Eigen::VectorXd::Zero(19);
    mQd = Eigen::VectorXd::Zero(18);
    mQdd = Eigen::VectorXd::Zero(18);
    mResidual = Eigen::VectorXd::Zero(18);
    mTau = Eigen::VectorXd::Zero(18);

    mEstState = Eigen::VectorXd::Zero(12);
    mState = Eigen::VectorXd::Zero(12);
    mErrorCov = Eigen::Matrix<double,12,12>::Identity();
    mKalmanGain = Eigen::Matrix<double,12,6>::Identity();
    mMeasureMat.setZero();
    mMeasurements = Eigen::VectorXd::Zero(6);
    tempIMU = Eigen::VectorXd::Zero(3);
    tempPrevIMU = Eigen::VectorXd::Zero(3);
    tempPrevV = Eigen::VectorXd::Zero(3);

    /// leg kinematics
    mPos << 0,0,0.076;
    mPrevPos << 0,0,0.076;
    for(int leg = 0; leg < 4; leg++)
    {
        mGlobalFootPos[leg].setZero();
    }

//    std::cout << "Degree of freedom overview : " << std::endl;
//    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*mModel);
//
//    std::cout << "Model Hierarchy:" << std::endl;
//    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*mModel);
//
//    std::cout << "q : " << mModel->q_size << ", qdot : " << mModel->qdot_size << std::endl;

//    for( int i=0; i<18; i++)
//    {
//        std::cout << "body name " << i << " : "<< mModel->GetBodyName(i) << std::endl;
//    }
//    std::cout << mGain << std::endl;
}

void SimulStateEstimator::StateEstimatorFunction()
{
    updateState();
    updateRBDL();
    getJointState();
    getRobotAngulerState();
    getRobotFootPosition();
    getContactState();
    getRobotLinearState();
}

void SimulStateEstimator::updateState()
{
    mPosition = mRobot->getGeneralizedCoordinate();
    mVelocity = mRobot->getGeneralizedVelocity();
    for(int i=0; i<18; i++)
    {
        mGeneralizedForce[i] = mRobot->getGeneralizedForce()[i];
    }
}

void SimulStateEstimator::getJointState()
{
    for (int idx=0; idx<MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = mPosition[idx+7];
        sharedMemory->motorVelocity[idx] = mVelocity[idx+6];
    }
}

void SimulStateEstimator::getRobotAngulerState()
{
    for(int idx=0; idx<3; idx++)
    {
        mGlobalBaseEulerVelocity[idx] = mVelocity[idx+3];
    }
    for(int idx=0; idx<4; idx++)
    {
        sharedMemory->baseQuartPosition[idx] = mPosition[idx+3];
    }
    mGlobalBaseEulerVelocity = GetBaseRotationMatInverse(sharedMemory->baseQuartPosition)*mGlobalBaseEulerVelocity;
    sharedMemory->baseEulerVelocity[0] = mGlobalBaseEulerVelocity[0];
    sharedMemory->baseEulerVelocity[1] = mGlobalBaseEulerVelocity[1];
    sharedMemory->baseEulerVelocity[2] = mGlobalBaseEulerVelocity[2];

    TransformQuat2Euler(sharedMemory->baseQuartPosition, mRawEulerAngle);

    if(tempV[2] > 3.14 && mRawEulerAngle[2] < 3.1/2.0)
    {
        mMulti++;
    }
    if(tempV[2] < -3.14 && mRawEulerAngle[2] > -3.1/2.0)
    {
        mMulti--;
    }

    tempV[2] = mRawEulerAngle[2];
    sharedMemory->baseEulerPosition[0] = mRawEulerAngle[0];
    sharedMemory->baseEulerPosition[1] = mRawEulerAngle[1];
    sharedMemory->baseEulerPosition[2] = mRawEulerAngle[2] + mMulti * 2.0 * 3.141592;
}

void SimulStateEstimator::getRobotFootPosition()
{
    TransMatBody2Foot(&mTransMat[LF_IDX], LF_IDX,
                      sharedMemory->motorPosition[LF_IDX*3],
                      sharedMemory->motorPosition[LF_IDX*3+1],
                      sharedMemory->motorPosition[LF_IDX*3+2]);
    TransMatBody2Foot(&mTransMat[RF_IDX], RF_IDX,
                      sharedMemory->motorPosition[RF_IDX*3],
                      sharedMemory->motorPosition[RF_IDX*3+1],
                      sharedMemory->motorPosition[RF_IDX*3+2]);
    TransMatBody2Foot(&mTransMat[LB_IDX], LB_IDX,
                      sharedMemory->motorPosition[LB_IDX*3],
                      sharedMemory->motorPosition[LB_IDX*3+1],
                      sharedMemory->motorPosition[LB_IDX*3+2]);
    TransMatBody2Foot(&mTransMat[RB_IDX], RB_IDX,
                      sharedMemory->motorPosition[RB_IDX*3],
                      sharedMemory->motorPosition[RB_IDX*3+1],
                      sharedMemory->motorPosition[RB_IDX*3+2]);

    Mat3<double> Rot = GetBaseRotationMat(sharedMemory->baseQuartPosition);
    for (int leg=0; leg<4; leg++)
    {
        sharedMemory->bodyFootPosition[leg] = mTransMat[leg].block(0,3,3,1);
        sharedMemory->globalFootPosition[leg] = sharedMemory->basePosition + Rot*sharedMemory->bodyFootPosition[leg];
    }
}

void SimulStateEstimator::getRobotLinearState()
{
    Mat3<double> Rot = GetBaseRotationMatInverse(sharedMemory->baseQuartPosition);
    Vec3<double> tempVec[2];
    for (int idx=0; idx<3; idx++)
    {
        sharedMemory->basePosition[idx] = mPosition[idx];
        sharedMemory->baseVelocity[idx] = mVelocity[idx];
        sharedMemory->testBasePos[idx] = mPosition[idx];
        sharedMemory->testBaseVel[idx] = mVelocity[idx];
    }
//
//    this->doLegKinematics();
//
    tempVec[0] = Rot * sharedMemory->bodyFootPosition[LF_IDX];
    tempVec[1] = Rot * sharedMemory->bodyFootPosition[RF_IDX];

//    std::cout<<"[GAIT TIMING] local time: " << std::setw(8) << std::right << sharedMemory->localTime
//             <<", [LF, RF] : " <<sharedMemory->gaitTable[0] <<", " <<sharedMemory->gaitTable[1]
//             <<" [position] : "<< std::setw(12) << std::right << mPos[0]<< std::setw(13) << std::right << mPos[1]<< std::setw(12) << std::right << mPos[2]
//            <<", [Body foot pos] [LF] : "
//            << std::setw(10) << std::right << tempVec[0][0]
//            << std::setw(11) << std::right << tempVec[0][1]
//            << std::setw(11) << std::right << tempVec[0][2]
//            << ", [RF] : "
//            << std::setw(10) << std::right << tempVec[1][0]
//            << std::setw(11) << std::right << tempVec[1][1]
//            << std::setw(11) << std::right << tempVec[1][2] << std::endl;

//    this->tempImuUpdate();
//    this->doEKF();

//    for(int i=0; i<3; i++)
//    {
//        sharedMemory->basePosition[i] = mEstState[i];
//        sharedMemory->baseVelocity[i] = mEstState[i+3];
//        sharedMemory->testBasePos[i] = mPosition[i];
//        sharedMemory->testBaseVel[i] = mVelocity[i];
//    }
}

void SimulStateEstimator::getContactState()
{
//    mTau.setZero();
//    double res[4];
//    RigidBodyDynamics::CompositeRigidBodyAlgorithm(* mModel, mQ, mH, true);
//    RigidBodyDynamics::InverseDynamics(*mModel, mQ, mQd, Eigen::VectorXd::Zero(18),mTau);
//
//    mBeta = mTau - (mH - mPrevH) / mDt * mQd;
//    mMomentum = mMomentum + mCalcTorque * mDt - mBeta * mDt + mResidual * mDt;
//    mResidual = mGain * (-mMomentum + mH * mQd);
//
//    res[LF_IDX] = sqrt(pow(mResidual[6],2) + pow(mResidual[7],2) + pow(mResidual[8],2));
//    res[LB_IDX] = sqrt(pow(mResidual[9],2) + pow(mResidual[10],2) + pow(mResidual[11],2));
//    res[RF_IDX] = sqrt(pow(mResidual[12],2) + pow(mResidual[13],2) + pow(mResidual[14],2));
//    res[RB_IDX] = sqrt(pow(mResidual[15],2) + pow(mResidual[16],2) + pow(mResidual[17],2));
//
//    mPrevH = mH;
//    mPrevQ = mQ;
//    mPrevQd = mQd;
//
//    if(mbContactState[LF_IDX] == 1 && res[LF_IDX] < 4)
//    {
//        mbContactState[LF_IDX] = 0;
//    }
//    else if(mbContactState[LF_IDX] == 0 && res[LF_IDX] > 8)
//    {
//        mbContactState[LF_IDX] = 1;
//    }
//
//    if(mbContactState[RF_IDX] == 1 && res[RF_IDX] < 4)
//    {
//        mbContactState[RF_IDX] = 0;
//    }
//    else if(mbContactState[RF_IDX] == 0 && res[RF_IDX] > 8)
//    {
//        mbContactState[RF_IDX] = 1;
//    }
//
//    if(mbContactState[LB_IDX] == 1 && res[LB_IDX] < 4)
//    {
//        mbContactState[LB_IDX] = 0;
//    }
//    else if(mbContactState[LB_IDX] == 0 && res[LB_IDX] >10)
//    {
//        mbContactState[LB_IDX] = 1;
//    }
//
//    if(mbContactState[RB_IDX] == 1 && res[RB_IDX] < 4)
//    {
//        mbContactState[RB_IDX] = 0;
//    }
//    else if(mbContactState[RB_IDX] == 0 && res[RB_IDX] >10)
//    {
//        mbContactState[RB_IDX] = 1;
//    }

    for(int i=0; i<4; i++)
    {
        mbContactState[i] = sharedMemory->gaitTable[i];
        sharedMemory->contactState[i] = mbContactState[i];
    }
}

void SimulStateEstimator::updateRBDL()
{
    mQ[0] = sharedMemory->basePosition[0];
    mQ[1] = sharedMemory->basePosition[1];
    mQ[2] = sharedMemory->basePosition[2];

    mQ[3] = sharedMemory->baseQuartPosition[1]; /// q_x
    mQ[4] = sharedMemory->baseQuartPosition[2]; /// q_y
    mQ[5] = sharedMemory->baseQuartPosition[3]; /// q_z
    mQ[18] = sharedMemory->baseQuartPosition[0]; /// q_w

    mQ[6] = sharedMemory->motorPosition[0];
    mQ[7] = sharedMemory->motorPosition[1];
    mQ[8] = sharedMemory->motorPosition[2];

    mQ[9] = sharedMemory->motorPosition[6];
    mQ[10] = sharedMemory->motorPosition[7];
    mQ[11] = sharedMemory->motorPosition[8];

    mQ[12] = sharedMemory->motorPosition[3];
    mQ[13] = sharedMemory->motorPosition[4];
    mQ[14] = sharedMemory->motorPosition[5];

    mQ[15] = sharedMemory->motorPosition[9];
    mQ[16] = sharedMemory->motorPosition[10];
    mQ[17] = sharedMemory->motorPosition[11];

    mQd[0] = sharedMemory->baseVelocity[0]; /// base_TX
    mQd[1] = sharedMemory->baseVelocity[1]; /// base_TY
    mQd[2] = sharedMemory->baseVelocity[2]; /// base_TZ

    mQd[3] = sharedMemory->baseEulerVelocity[2]; /// base_RZ
    mQd[4] = sharedMemory->baseEulerVelocity[1]; /// base_RY
    mQd[5] = sharedMemory->baseEulerVelocity[0]; /// base_RX

    mQd[6] = sharedMemory->motorVelocity[0]; /// LF_hip_RX
    mQd[7] = sharedMemory->motorVelocity[1];/// LF_thigh_RY
    mQd[8] = sharedMemory->motorVelocity[2];/// LF_calf_RY

    mQd[9] = sharedMemory->motorVelocity[6]; /// LB_hip_RX
    mQd[10] = sharedMemory->motorVelocity[7];/// LB_thigh_RY
    mQd[11] = sharedMemory->motorVelocity[8];/// LB_calf_RY

    mQd[12] = sharedMemory->motorVelocity[3];/// RF_hip_RX
    mQd[13] = sharedMemory->motorVelocity[4];/// RF_thigh_RX
    mQd[14] = sharedMemory->motorVelocity[5];/// RF_calf_RX

    mQd[15] = sharedMemory->motorVelocity[9];/// RB_hip_RX
    mQd[16] = sharedMemory->motorVelocity[10];/// RB_thigh_RX
    mQd[17] = sharedMemory->motorVelocity[11];/// RB_calf_RX

    mCalcTorque[0] = 0;
    mCalcTorque[1] = 0;
    mCalcTorque[2] = 0;

    mCalcTorque[3] = 0;
    mCalcTorque[4] = 0;
    mCalcTorque[5] = 0;

    mCalcTorque[6] = sharedMemory->motorDesiredTorque[0];
    mCalcTorque[7] = sharedMemory->motorDesiredTorque[1];
    mCalcTorque[8] = sharedMemory->motorDesiredTorque[2];

    mCalcTorque[9] = sharedMemory->motorDesiredTorque[6];
    mCalcTorque[10] = sharedMemory->motorDesiredTorque[7];
    mCalcTorque[11] = sharedMemory->motorDesiredTorque[8];

    mCalcTorque[12] = sharedMemory->motorDesiredTorque[3];
    mCalcTorque[13] = sharedMemory->motorDesiredTorque[4];
    mCalcTorque[14] = sharedMemory->motorDesiredTorque[5];

    mCalcTorque[15] = sharedMemory->motorDesiredTorque[9];
    mCalcTorque[16] = sharedMemory->motorDesiredTorque[10];
    mCalcTorque[17] = sharedMemory->motorDesiredTorque[11];
}

void SimulStateEstimator::setOffset()
{
    mOffset[LF_IDX] = -sharedMemory->bodyFootPosition[LF_IDX];
    mOffset[RF_IDX] = -sharedMemory->bodyFootPosition[RF_IDX];
    mOffset[LB_IDX] = -sharedMemory->bodyFootPosition[LB_IDX];
    mOffset[RB_IDX] = -sharedMemory->bodyFootPosition[RB_IDX];
}

void SimulStateEstimator::doEKF()
{
    Vec3<double> globalBodyFootPosition[4];
    Mat3<double> Rot = GetBaseRotationMatInverse(sharedMemory->baseQuartPosition);
    Vec3<double> bodyFootPosition[2];
    for(int leg=0; leg<2; leg++)
    {
        globalBodyFootPosition[leg] = Rot.inverse() * sharedMemory->bodyFootPosition[leg];
        bodyFootPosition[leg] = sharedMemory->bodyFootPosition[leg];
    }

    if(mbIsFirstFiltering)
    {
        mState[0] = 0;
        mState[1] = 0;
        mState[2] = 0.076;

        mState[3] = 0;
        mState[4] = 0;
        mState[5] = 0;

        mState[6] = globalBodyFootPosition[0][0];
        mState[7] = globalBodyFootPosition[0][1];
        mState[8] = globalBodyFootPosition[0][2];

        mState[9] = globalBodyFootPosition[1][0];
        mState[10] = globalBodyFootPosition[1][1];
        mState[11] = globalBodyFootPosition[1][2];

        for(int i=0; i<12; i++)
        {
            mEstState[i] = mState[i];
        }

        ///predict matrix (A)
        mPredictMat = Eigen::Matrix<double,12,12>::Identity();
        for(int i=0; i<3; i++)
        {
            mPredictMat(i,i+3) = mDt;
        }
        ///input matrix (B)
        mInputMat.setZero();
        for(int i=0; i<3; i++)
        {
            mInputMat(i,i) = (1.0/2.0) * mDt * mDt;
            mInputMat(i+3,i) = mDt;
        }

        ///error cov
        for(int i=0; i<12; i++)
        {
            mErrorCov(i,i) = 0.005*0.005;
        }

        mNoiseQ = Eigen::Matrix<double,12,12>::Identity() * 1.2;
        mNoiseR = Eigen::Matrix<double,6,6>::Identity() * 0.01;

        mbIsFirstFiltering = false;
    }

    for(int i=0; i<2; i++)
    {
        if(mbContactState[i])
        {
            if(mbIsFirstCon[i])
            {
                mEstState[3*i + 6] = mEstState[0] + globalBodyFootPosition[i][0];
                mEstState[3*i + 7] = mEstState[1] + globalBodyFootPosition[i][1];
                mEstState[3*i + 8] = mEstState[2] + globalBodyFootPosition[i][2];
                mbIsFirstCon[i] = 0;
            }
            mNoiseQ(3*i+6,3*i+6) = 1.2;
            mNoiseQ(3*i+7,3*i+7) = 1.2;
            mNoiseQ(3*i+8,3*i+8) = 1.2;
        }
        else
        {
            mbIsFirstCon[i] = 1;
            mNoiseQ(3*i+6,3*i+6) = 1000000;
            mNoiseQ(3*i+7,3*i+7) = 1000000;
            mNoiseQ(3*i+8,3*i+8) = 1000000;
        }
    }

    ///measurement matrix update
    for(int roww=0; roww<3; roww++)
    {
        for(int column=0; column<3; column++)
        {
            mMeasureMat(roww,column) =
            mMeasureMat(roww + 3,column) = -Rot(roww,column);

            mMeasureMat(roww,column + 6) =
            mMeasureMat(roww + 3,column + 9) = Rot(roww,column);
        }
    }

    ///time update
    mState = mPredictMat * mEstState + mInputMat * tempIMU;
    mErrorCov = mPredictMat * mErrorCov * mPredictMat.transpose() + mNoiseQ;

    ///calculate kalman gain
    mKalmanGain = mErrorCov * mMeasureMat.transpose() * (mMeasureMat * mErrorCov * mMeasureMat.transpose() + mNoiseR).inverse();

    ///measurement update
    for(int leg=0; leg<2; leg++)
    {
        for(int row=0; row<3; row++)
        {
            mMeasurements(3*leg+row) = bodyFootPosition[leg][row];
        }
    }
    mEstState = mState + mKalmanGain * (mMeasurements - mMeasureMat * mState);

    ///error covariance update
    mErrorCov = mErrorCov - mKalmanGain * mMeasureMat * mErrorCov;

    ///show the outputs
//    this->showOutputs();

}

///
void SimulStateEstimator::doLegKinematics()
{
    Mat3<double> rotWorld2Body;
    Mat3<double> jacobian;
    Vec3<double> bodyFootPos[4];
    Vec3<double> bodyVel;
    Vec3<double> motorPositionLF;
    Vec3<double> motorPositionRF;

    Vec3<double> jointVelLF;
    Vec3<double> jointVelRF;
    Vec3<double> anglarVel;

    bodyFootPos[LF_IDX] = sharedMemory->bodyFootPosition[LF_IDX];
    bodyFootPos[RF_IDX] = sharedMemory->bodyFootPosition[RF_IDX];
    rotWorld2Body = GetBaseRotationMat(sharedMemory->baseQuartPosition);

    jointVelLF[0] = mVelocity[6];
    jointVelLF[1] = mVelocity[7];
    jointVelLF[2] = mVelocity[8];
    jointVelRF[0] = mVelocity[9];
    jointVelRF[1] = mVelocity[10];
    jointVelRF[2] = mVelocity[11];

    motorPositionLF[0]=sharedMemory->motorPosition[LF_IDX*3];
    motorPositionLF[1]=sharedMemory->motorPosition[LF_IDX*3 + 1];
    motorPositionLF[2]=sharedMemory->motorPosition[LF_IDX*3 + 2];
    motorPositionRF[0]=sharedMemory->motorPosition[RF_IDX*3];
    motorPositionRF[1]=sharedMemory->motorPosition[RF_IDX*3 + 1];
    motorPositionRF[2]=sharedMemory->motorPosition[RF_IDX*3 + 2];

    anglarVel[0];
    anglarVel[0];
    anglarVel[0];

//    std::cout<<"[Leg kinematics] bodyFotPos[LF] : "<<bodyFootPos[LF_IDX]<<std::endl;
//    std::cout<<"[Leg kinematics] bodyFotPos[RF] : "<<bodyFootPos[RF_IDX]<<std::endl;
//    std::cout<<"[Leg kinematics] Rot : "<<rotWorld2Body<<std::endl;
    if(mbIsFirstRun)
    {
        mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
        mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
        mbIsFirstRun = false;
//        std::cout <<"[initial global foot], [LF] : "
//                << std::setw(14) << std::right <<mGlobalFootPos[LF_IDX][0]
//                << std::setw(14) << std::right <<mGlobalFootPos[LF_IDX][1]
//                << std::setw(14) << std::right <<mGlobalFootPos[LF_IDX][2]
//                << ", [RF] : "
//                << std::setw(14) << std::right <<mGlobalFootPos[RF_IDX][0]
//                << std::setw(14) << std::right <<mGlobalFootPos[RF_IDX][1]
//                << std::setw(14) << std::right <<mGlobalFootPos[RF_IDX][2] << std::endl;
    }

        if ((mbContactState[LF_IDX] == 1) && (mbContactState[RF_IDX] == 1))
        {
            if (!(mbIsFirstCon[LF_IDX]) & mbIsFirstCon[RF_IDX]) {
                mPos = mGlobalFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX];
                mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
                GetJacobian(jacobian,motorPositionLF,LF_IDX);
                bodyVel = rotWorld2Body * GetSkew(bodyFootPos[LF_IDX]) *   - rotWorld2Body * jacobian * jointVelLF;
//            std::cout<<"[BOTH 1]";
//            std::cout << std::setw(15) << std::right << mPos[0]
//                      << std::setw(15) << std::right << mPos[1]
//                      << std::setw(15) << std::right << mPos[2] << std::endl;
            } else if (mbIsFirstCon[LF_IDX] & !(mbIsFirstCon[RF_IDX])) {
                mPos = mGlobalFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX];
                mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
                GetJacobian(jacobian,motorPositionRF,RF_IDX);
                bodyVel = rotWorld2Body * bodyFootPos[RF_IDX] - rotWorld2Body * jacobian * jointVelRF;
//            std::cout<<"[BOTH 2]";
//            std::cout << std::setw(15) << std::right << mPos[0]
//                      << std::setw(15) << std::right << mPos[1]
//                      << std::setw(15) << std::right << mPos[2] << std::endl;
            } else if (mbIsFirstCon[LF_IDX] & mbIsFirstCon[RF_IDX]) {
//            std::cout<<"[ERROR]"<<std::endl;
                mPos = (mGlobalFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX]) / 2
                       + (mGlobalFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX]) / 2;
            }
        }
        else if (mbContactState[LF_IDX])
        {
            if (mbIsFirstCon[LF_IDX]) {
                mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
                mbIsFirstCon[LF_IDX] = false;
                mbIsFirstCon[RF_IDX] = true;
            }
            mPos = mGlobalFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX];
            mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
            GetJacobian(jacobian,motorPositionLF,LF_IDX);
            bodyVel = rotWorld2Body * bodyFootPos[LF_IDX] - rotWorld2Body * jacobian * jointVelLF;
//        std::cout<<"[LF]";
//        std::cout << std::setw(15) << std::right << mPos[0]
//                  << std::setw(15) << std::right << mPos[1]
//                  << std::setw(15) << std::right << mPos[2] << std::endl;

        }
        else if (mbContactState[RF_IDX])
        {
            if (mbIsFirstCon[RF_IDX]) {
                mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
                mbIsFirstCon[RF_IDX] = false;
                mbIsFirstCon[LF_IDX] = true;
            }
            mPos = mGlobalFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX];
            mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
            GetJacobian(jacobian,motorPositionRF,RF_IDX);
            bodyVel = rotWorld2Body * bodyFootPos[RF_IDX] - rotWorld2Body * jacobian * jointVelRF;
//        std::cout<<"[RF]";
//        std::cout << std::setw(15) << std::right << mPos[0]
//                  << std::setw(15) << std::right << mPos[1]
//                  << std::setw(15) << std::right << mPos[2] << std::endl;
        }

//        bodyVel = (mPos - mPrevPos) / ESTIMATOR_dT;

        bodyVel = rotWorld2Body * bodyFootPos[LF_IDX] - rotWorld2Body * jacobian * jointVelLF;

        mPrevPos = mPos;

        for (int idx = 0; idx < 3; idx++)
        {
            sharedMemory->basePosition[idx] = mPos[idx];
            sharedMemory->baseVelocity[idx] = bodyVel[idx];
//        sharedMemory->testBasePos[idx] = mPos[idx];
//        sharedMemory->testBaseVel[idx] = bodyVel[idx];
        }

}


///

/**
 * just for simulation.(to get acceleration)
 */
void SimulStateEstimator::tempImuUpdate()
{
    tempIMU = (sharedMemory->baseVelocity - tempPrevV)/mDt;

    if(tempIMU[0] == 0 && tempIMU[1] == 0 && tempIMU[2] == 0)
    {
        for(int i=0; i<3; i++)
        {
            tempIMU[i] = tempPrevIMU[i];
        }
    }

    for(int i=0; i<3; i++)
    {
        tempPrevIMU[i] = tempIMU[i];
        sharedMemory->baseAcceleration[i] = tempIMU[i];
        tempPrevV[i] = sharedMemory->baseVelocity[i];
    }
}

void SimulStateEstimator::showOutputs()
{
    for(int i=0; i<12; i++)
    {
        std::cout << std::setw(14) << std::right << mEstState[i];
    }
    std::cout << std::endl;
}