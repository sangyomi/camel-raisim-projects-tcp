//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

StateEstimator::StateEstimator(RigidBodyDynamics::Model* model)
    : mDt(0.001)
    , mGain(Eigen::Matrix<double,18,18>::Identity() * 50)
    , mModel(model)
    , mbIsFirstFiltering(true)
    , mbIsFirstRun(true)
{
    for(int i=0; i<4; i++)
    {
        mbContactState[i] = true;
        mbIsFirstCon[i] = true;
        mbIsFirstConLegK[i] = true;
    }
    mBeta = Eigen::VectorXd::Zero(18);
    mCalcTorque = Eigen::VectorXd::Zero(18);
    mH = Eigen::MatrixXd::Zero(18,18);
    mMomentum = Eigen::VectorXd::Zero(18);
    mPrevH = Eigen::MatrixXd::Zero(18,18);
    mPrevQ = Eigen::VectorXd::Zero(19);
    mPrevQd = Eigen::VectorXd::Zero(18);
    mQ = Eigen::VectorXd::Zero(19);
    mQd = Eigen::VectorXd::Zero(18);
    mResidual = Eigen::VectorXd::Zero(18);
    mTau = Eigen::VectorXd::Zero(18);

    mEstState = Eigen::VectorXd::Zero(12);
    mState = Eigen::VectorXd::Zero(12);
    mErrorCov = Eigen::Matrix<double,12,12>::Identity();
    mKalmanGain = Eigen::Matrix<double,12,6>::Identity();
    mMeasureMat.setZero();
    mMeasurements = Eigen::VectorXd::Zero(6);
    tempPrevIMU = Eigen::VectorXd::Zero(3);
    tempPrevV = Eigen::VectorXd::Zero(3);

    /// leg kinematics
    mPos << 0,0,0.055;
    mPrevPos << 0,0,0.055;
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

//    std::cout << mGain << std::endl;
}

void StateEstimator::StateEstimatorFunction()
{
    if (sharedMemory->motorLFState && sharedMemory->motorRFState && sharedMemory->motorLBState && sharedMemory->motorRBState)
    {
        updateState();
//        updateRBDL();
        getRobotFootPosition();
        getContactState();
//        getRobotLinearState();
    }
}
void StateEstimator::updateRBDL()
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

    mQ[9] = sharedMemory->motorPosition[3];
    mQ[10] = sharedMemory->motorPosition[4];
    mQ[11] = sharedMemory->motorPosition[5];

    mQ[12] = sharedMemory->motorPosition[6];
    mQ[13] = sharedMemory->motorPosition[7];
    mQ[14] = sharedMemory->motorPosition[8];

    mQ[15] = sharedMemory->motorPosition[9];
    mQ[16] = sharedMemory->motorPosition[10];
    mQ[17] = sharedMemory->motorPosition[11];

    mQd[0] = sharedMemory->baseVelocity[0]; /// base_TX
    mQd[1] = sharedMemory->baseVelocity[1]; /// base_TY
    mQd[2] = sharedMemory->baseVelocity[2]; /// base_TZ

    mQd[3] = sharedMemory->baseEulerVelocity[2]; /// base_RZ
    mQd[4] = sharedMemory->baseEulerVelocity[1]; /// base_RY
    mQd[5] = sharedMemory->baseEulerVelocity[0]; /// base_RX

    mQd[6] = sharedMemory->motorVelocity[0]; /// FL_hip_RX
    mQd[7] = sharedMemory->motorVelocity[1];/// FL_thigh_RY
    mQd[8] = sharedMemory->motorVelocity[2];/// FL_calf_RY

    mQd[9] = sharedMemory->motorVelocity[3]; /// FR_hip_RX
    mQd[10] = sharedMemory->motorVelocity[4];/// FR_thigh_RY
    mQd[11] = sharedMemory->motorVelocity[5];/// FR_calf_RY

    mQd[12] = sharedMemory->motorVelocity[6];/// HL_hip_RX
    mQd[13] = sharedMemory->motorVelocity[7];/// HL_thigh_RX
    mQd[14] = sharedMemory->motorVelocity[8];/// HL_calf_RX

    mQd[15] = sharedMemory->motorVelocity[9];/// HR_hip_RX
    mQd[16] = sharedMemory->motorVelocity[10];/// HR_thigh_RX
    mQd[17] = sharedMemory->motorVelocity[11];/// HR_calf_RX

    mCalcTorque[0] = 0;
    mCalcTorque[1] = 0;
    mCalcTorque[2] = 0;

    mCalcTorque[3] = 0;
    mCalcTorque[4] = 0;
    mCalcTorque[5] = 0;

    mCalcTorque[6] = sharedMemory->motorDesiredTorque[0];
    mCalcTorque[7] = sharedMemory->motorDesiredTorque[1];
    mCalcTorque[8] = sharedMemory->motorDesiredTorque[2];

    mCalcTorque[9] = sharedMemory->motorDesiredTorque[3];
    mCalcTorque[10] = sharedMemory->motorDesiredTorque[4];
    mCalcTorque[11] = sharedMemory->motorDesiredTorque[5];

    mCalcTorque[12] = sharedMemory->motorDesiredTorque[6];
    mCalcTorque[13] = sharedMemory->motorDesiredTorque[7];
    mCalcTorque[14] = sharedMemory->motorDesiredTorque[8];

    mCalcTorque[15] = sharedMemory->motorDesiredTorque[9];
    mCalcTorque[16] = sharedMemory->motorDesiredTorque[10];
    mCalcTorque[17] = sharedMemory->motorDesiredTorque[11];
}

void StateEstimator::updateState()
{
    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
    }
    for(int idx=0; idx<3; idx++)
    {
        mAcceleration[idx] = sharedMemory->baseAcceleration[idx];
    }
}

void StateEstimator::getContactState()
{
//    double res[4];
//    RigidBodyDynamics::CompositeRigidBodyAlgorithm(* mModel, mQ, mH, true);
//    RigidBodyDynamics::InverseDynamics(*mModel, mQ, mQd, Eigen::VectorXd::Zero(18),mTau);
//
//    mBeta = mTau - (mH - mPrevH) / mDt * mQd;
//    mMomentum = mMomentum + mCalcTorque * mDt - mBeta * mDt + mResidual * mDt;
//    mResidual = mGain * (-mMomentum + mH * mQd);
//
//    res[LF_IDX] = sqrt(pow(mResidual[6],2) + pow(mResidual[7],2) + pow(mResidual[8],2));
//    res[RF_IDX] = sqrt(pow(mResidual[9],2) + pow(mResidual[10],2) + pow(mResidual[11],2));
//    res[LB_IDX] = sqrt(pow(mResidual[12],2) + pow(mResidual[13],2) + pow(mResidual[14],2));
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
        if(sharedMemory->gaitTable[i] == 1)
        {
            mbContactState[i] = true;
        }
        else
        {
            mbContactState[i] = false;
        }
//        mbContactState[i] = sharedMemory->gaitTable[i];
//        std::cout<<"[SE] contact : "<<mbContactState[i]<<std::endl;
//        std::cout<<"[SE] gait table : "<<sharedMemory->gaitTable[i]<<std::endl;
//        std::cout<<"[SE] is first contact : "<<mbIsFirstConLegK[i]<<std::endl;
    }
}

void StateEstimator::getRobotFootPosition()
{
    TransMatBody2Foot(&mTransMat[0], LF_IDX,
                      sharedMemory->motorPosition[0],
                      sharedMemory->motorPosition[1],
                      sharedMemory->motorPosition[2]);
    TransMatBody2Foot(&mTransMat[1], RF_IDX,
                      sharedMemory->motorPosition[3],
                      sharedMemory->motorPosition[4],
                      sharedMemory->motorPosition[5]);
    TransMatBody2Foot(&mTransMat[2], LB_IDX,
                      sharedMemory->motorPosition[6],
                      sharedMemory->motorPosition[7],
                      sharedMemory->motorPosition[8]);
    TransMatBody2Foot(&mTransMat[3], RB_IDX,
                      sharedMemory->motorPosition[9],
                      sharedMemory->motorPosition[10],
                      sharedMemory->motorPosition[11]);

    Mat3<double> Rot = GetBaseRotationMat(sharedMemory->baseQuartPosition);
    for (int leg=0; leg<4; leg++)
    {
        sharedMemory->bodyFootPosition[leg] = mTransMat[leg].block(0,3,3,1);
        sharedMemory->globalFootPosition[leg] = sharedMemory->basePosition + Rot*sharedMemory->bodyFootPosition[leg];
    }
}

void StateEstimator::getResidual(double *residual)
{
}

void StateEstimator::getRobotLinearState()
{
    this->doEKF();
//    this->doLegKinematics();


    for(int i=0; i<3; i++)
    {
//        sharedMemory->basePosition[i] = mEstState[i];
//        sharedMemory->baseVelocity[i] = mEstState[i+3];
    }

//    sharedMemory->basePosition[0] = mPos[0];
//    sharedMemory->basePosition[1] = mPos[1];

//    sharedMemory->basePosition[0] = mPos[0];
//    sharedMemory->basePosition[1] = mPos[1];
//    sharedMemory->basePosition[2] = mPos[2];
    sharedMemory->baseVelocity[0] = mEstState[3];
    sharedMemory->baseVelocity[1] = mEstState[4];
    sharedMemory->baseVelocity[2] = mEstState[5];

}

///
void StateEstimator::doLegKinematics()
{
//    std::cout<<"[Leg kinematics] local time : "<<sharedMemory->localTime<<std::endl;
    Mat3<double> rotWorld2Body;
    Vec3<double> bodyFootPos[4];
    Vec3<double> bodyVel;
    bodyFootPos[LF_IDX] = sharedMemory->bodyFootPosition[LF_IDX];
    bodyFootPos[RF_IDX] = sharedMemory->bodyFootPosition[RF_IDX];
    rotWorld2Body = GetBaseRotationMat(sharedMemory->baseQuartPosition);

//    std::cout<<"[Leg kinematics] bodyFotPos[LF] : "<<bodyFootPos[LF_IDX]<<std::endl;
//    std::cout<<"[Leg kinematics] bodyFotPos[RF] : "<<bodyFootPos[RF_IDX]<<std::endl;
//    std::cout<<"[Leg kinematics] Rot : "<<rotWorld2Body<<std::endl;
    if(mbIsFirstRun)
    {
        mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body*bodyFootPos[LF_IDX];
        mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body*bodyFootPos[RF_IDX];
        mbIsFirstRun = false;
    }
    if(mbContactState[LF_IDX] && mbContactState[RF_IDX])
    {
//        std::cout<<"[CONTACT]"<<std::endl;
        if(!(mbIsFirstConLegK[LF_IDX]) && mbIsFirstConLegK[RF_IDX])
        {
            mPos = mGlobalFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX];
            mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
//            std::cout<<"[BOTH 1]";
            std::cout << std::setw(15) << std::right << mPos[0]
                      << std::setw(15) << std::right << mPos[1]
                      << std::setw(15) << std::right << mPos[2] << std::endl;
        }
        else if(mbIsFirstConLegK[LF_IDX] && !(mbIsFirstConLegK[RF_IDX]))
        {
            mPos = mGlobalFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX];
            mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
//            std::cout<<"[BOTH 2]";
            std::cout << std::setw(15) << std::right << mPos[0]
                      << std::setw(15) << std::right << mPos[1]
                      << std::setw(15) << std::right << mPos[2] << std::endl;
        }
        else if(mbIsFirstConLegK[LF_IDX] && mbIsFirstConLegK[RF_IDX])
        {
//            std::cout<<"[ERROR]"<<std::endl;
            mPos = (mGlobalFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX])/2
                   + (mGlobalFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX])/2;
        }
        else
        {
//            std::cout<<"[DEFAULT]"<<std::endl;
        }
    }
    else if(mbContactState[LF_IDX])
    {
//        std::cout<<"[DEFAULT1]"<<std::endl;
        if(mbIsFirstConLegK[LF_IDX])
        {
//            std::cout<<"[DEFAULT2]"<<std::endl;
            mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
            mbIsFirstConLegK[LF_IDX] = false;
            mbIsFirstConLegK[RF_IDX] = true;
        }
        mPos = mGlobalFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX];
//        mPos[2] += 0.025;
        mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
//        std::cout<<"[LF]";
//        std::cout << std::setw(15) << std::right << mPos[0]
//                  << std::setw(15) << std::right << mPos[1]
//                  << std::setw(15) << std::right << mPos[2] << std::endl;

    }
    else if(mbContactState[RF_IDX])
    {
        if(mbIsFirstConLegK[RF_IDX])
        {
            mGlobalFootPos[RF_IDX] = mPos + rotWorld2Body * bodyFootPos[RF_IDX];
            mbIsFirstConLegK[RF_IDX] = false;
            mbIsFirstConLegK[LF_IDX] = true;
        }
        mPos = mGlobalFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX];
//        mPos[2] += 0.025;
        mGlobalFootPos[LF_IDX] = mPos + rotWorld2Body * bodyFootPos[LF_IDX];
//        std::cout<<"[RF]";
//        std::cout << std::setw(15) << std::right << mPos[0]
//                  << std::setw(15) << std::right << mPos[1]
//                  << std::setw(15) << std::right << mPos[2] << std::endl;
    }
//    bodyVel = (rotWorld2Body.inverse() * mPos - rotWorld2Body.inverse() * mPrevPos)/ESTIMATOR_dT;

    bodyVel = (mPos - mPrevPos)/ESTIMATOR_dT;
    mPrevPos = mPos;

    for(int idx=0; idx<3; idx++)
    {
//        sharedMemory->testBasePos[idx] = mPos[idx];
//        sharedMemory->testBaseVel[idx] = bodyVel[idx];
//        sharedMemory->basePosition[idx] = mPos[idx];
//        sharedMemory->baseVelocity[idx] = bodyVel[idx];
    }
}
///

void StateEstimator::doEKF()
{
    Vec3<double> bodyFootPosition[4];
    Vec3<double> globalBodyFootPosition[4];
    Vec3<double> linearAcc;
    Mat3<double> Rot = GetBaseRotationMatInverse(sharedMemory->baseQuartPosition);
    for(int leg=0; leg<2; leg++)
    {
        bodyFootPosition[leg] = sharedMemory->bodyFootPosition[leg];
        globalBodyFootPosition[leg] = Rot.inverse() * sharedMemory->bodyFootPosition[leg];
    }

    linearAcc[0] = sharedMemory->baseAcceleration[0];
    linearAcc[1] = sharedMemory->baseAcceleration[1];
    linearAcc[2] = sharedMemory->baseAcceleration[2];
    linearAcc = Rot * linearAcc;

    if(mbIsFirstFiltering)
    {
        mState[0] = 0;
        mState[1] = 0;
        mState[2] = 0.055;

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
            mErrorCov(i,i) = 0.05*0.05;
        }

        mNoiseQ = Eigen::Matrix<double,12,12>::Identity() * 1.5;
        mNoiseR = Eigen::Matrix<double,6,6>::Identity() * 0.1;

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
            mNoiseQ(3*i+6,3*i+6) = 1.5;
            mNoiseQ(3*i+7,3*i+7) = 1.5;
            mNoiseQ(3*i+8,3*i+8) = 1.5;
        }
        else
        {
            mbIsFirstCon[i] = 1;
            mNoiseQ(3*i+6,3*i+6) = 100000;
            mNoiseQ(3*i+7,3*i+7) = 100000;
            mNoiseQ(3*i+8,3*i+8) = 100000;
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
    mState = mPredictMat * mEstState + mInputMat * linearAcc;


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

}

void StateEstimator::showOutputs()
{
    for(int i=0; i<12; i++)
    {
        std::cout << std::setw(14) << std::right << mEstState[i];
    }
    for(int i=0; i<3; i++)
    {
        std::cout << std::setw(14) << std::right << sharedMemory->baseAcceleration[i];
    }
    std::cout << std::endl;
}