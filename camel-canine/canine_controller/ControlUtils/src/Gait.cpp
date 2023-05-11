//
// Created by hs on 22. 8. 10.
//

#include <ControlUtils/Gait.hpp>

extern pSHM sharedMemory;

OffsetGait::OffsetGait(int mpcHorizon, uint64_t cyclePeriod, Vec4<int> offsets, Vec4<int> durations)
    : mOffsets(offsets.array())
    , mDurations(durations.array())
    , mHorizon(mpcHorizon)
    , mCyclePeriod(cyclePeriod)
{
}


OffsetGait::~OffsetGait() noexcept
{
}

void OffsetGait::getGaitTable()
{
    for(int i = 0; i < mHorizon; i++)
    {
        int iter = (i + sharedMemory->gaitIteration) % mCyclePeriod;
        Eigen::Array4i progress = iter - mOffsets;
        for(int j = 0; j < 4; j++)
        {
            if(progress[j] < 0)
            {
                progress[j] += mCyclePeriod;
            }
            if(progress[j] < mDurations[j])
            {
                mGaitTable[i*4 + j] = 1;
            }
            else
            {
                mGaitTable[i*4 + j] = 0;
            }
        }
    }
    for(int i = 0 ; i< MPC_HORIZON * 4 ; i++)
    {
        sharedMemory->gaitTable[i] = mGaitTable[i];
    }
}

void OffsetGait::getChangeGaitTable()
{
    for(int i = 0; i < mHorizon; i++)
    {
        if ((i + sharedMemory->gaitIteration) >= mCyclePeriod)
        {
            int iter = (i + sharedMemory->gaitIteration) % mCyclePeriod;
            Eigen::Array4i progress = iter - mOffsets;
            for(int j = 0; j < 4; j++)
            {
                if(progress[j] < 0)
                {
                    progress[j] += mCyclePeriod;
                }
                if(progress[j] < mDurations[j])
                {
                    sharedMemory->gaitTable[i*4 + j] = 1;
                }
                else
                {
                    sharedMemory->gaitTable[i*4 + j] = 0;
                }
            }
        }
    }

}