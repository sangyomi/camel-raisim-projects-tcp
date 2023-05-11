//
// Created by hs on 22. 10. 27.
//

#include <LowController/LowPDcontrol.hpp>

extern pSHM sharedMemory;

LowPDcontrol::LowPDcontrol()
    : mTorqueLimit(TORQUE_LIMIT)
    , bIsFirstRunTrot(true)
    , mSwingPgain{ 50, 50, 50 }
    , mSwingDgain{ 1, 1, 1 }
    , mStandPgain{15,15,15}
    , mStandDgain{1.5,1.5,1.5}
{
    mTorque->setZero();
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mReturnTorque[idx] = 0;
    }

}

void LowPDcontrol::DoControl()
{
    updateState();

    if (sharedMemory->gaitState != STAND)
    {
        if (bIsFirstRunTrot)
        {
            InitSwingTrjactory();
            bIsFirstRunTrot = false;
        }
    }
    setLegControl();
    setControlInput();
}

void LowPDcontrol::InitSwingTrjactory()
{
//    SwingLegTrajectory.UpdateTrajectory(sharedMemory->localTime);
}

void LowPDcontrol::updateState()
{
    for (int idx = 0; idx < 3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
    }

    for (int idx = 0; idx < 4; idx++)
    {
        mBaseQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
        mBodyFootPosition[idx] = sharedMemory->bodyFootPosition[idx];
        mGlobalFootPosition[idx] = sharedMemory->globalFootPosition[idx];
        for (int mt = 0; mt < 3; mt++)
        {
            mMotorPosition[idx][mt] = sharedMemory->motorPosition[idx * 3 + mt];
            mMotorVelocity[idx][mt] = sharedMemory->motorVelocity[idx * 3 + mt];
        }
    }
}

void LowPDcontrol::getJointPos(Vec3<double>& jointPos, Vec3<double> footPos)
{
    double d = sqrt(pow(footPos[0],2)+pow(footPos[2],2));
    double phi = acos(abs(footPos[0])/ d);
    double psi = acos(pow(d,2)/(2*LEN_THI*d));

    if (footPos[0] < 0)
        jointPos[1] = 1.57 - phi + psi;
    else if(footPos[0] == 0)
        jointPos[1] = psi;
    else
        jointPos[1] = phi + psi - 1.57;
    jointPos[2] = -acos((pow(d,2)-2*pow(LEN_CAL,2)) / (2*LEN_CAL*LEN_CAL));
}

void LowPDcontrol::setLegControl()
{
    Vec3<double> standFootPosition;
    standFootPosition << 0.0, 0.0, sharedMemory->baseDesiredPosition[2];
//    SwingLegTrajectory.GetPositionTrajectory(sharedMemory->localTime, mSwingFootPosition); //global
    getJointPos(mSwingJointPos, mSwingFootPosition);
    getJointPos(mStandJointPos, standFootPosition);

    for (int leg = 0; leg < 4; leg++)
    {
        if (sharedMemory->gaitTable[leg] == 0)
        {
            for(int mt=0; mt<3; mt++)
            {
                mLegTorque[leg][mt] = mSwingPgain[mt] * (mSwingJointPos[mt] - mMotorPosition[leg][mt])
                    + mSwingDgain[mt] * (mSwingJointVel[mt] - mMotorVelocity[leg][mt]);
            }
            sharedMemory->desiredFootPosition[leg] = mSwingFootPosition;
        }
        else
        {
            for(int mt=0; mt<3; mt++)
            {
                mLegTorque[leg][mt] = mStandPgain[mt] * (mStandJointPos[mt] - mMotorPosition[leg][mt])
                    + mStandDgain[mt] * (mStandJointVel[mt] - mMotorVelocity[leg][mt]);
            }
            sharedMemory->desiredFootPosition[leg] = standFootPosition;
        }
    }

    for(int idx=0; idx<4; idx++)
    {
        mTorque[idx][0] = mLegTorque[idx][0] + sharedMemory->mpcTorque[idx][0];
        mTorque[idx][1] = mLegTorque[idx][1] + sharedMemory->mpcTorque[idx][1];
        mTorque[idx][2] = mLegTorque[idx][2] + sharedMemory->mpcTorque[idx][2];
    }
}

void LowPDcontrol::setControlInput()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            if (mTorque[leg][motor] > mTorqueLimit)
            {
                mTorque[leg][motor] = mTorqueLimit;
            }
            else if (mTorque[leg][motor] < -mTorqueLimit)
            {
                mTorque[leg][motor] = -mTorqueLimit;
            }
            sharedMemory->motorDesiredTorque[leg*3+motor] = mTorque[leg][motor];
        }
    }
}

double* LowPDcontrol::GetTorque()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            mReturnTorque[leg * 3 + motor] = mTorque[leg][motor];
        }
    }
    return mReturnTorque;
}