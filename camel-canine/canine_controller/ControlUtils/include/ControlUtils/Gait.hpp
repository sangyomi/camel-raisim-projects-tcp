//
// Created by hs on 22. 8. 10.
//

#ifndef RAISIM_GAIT_H
#define RAISIM_GAIT_H

#include <iostream>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/SharedMemory.hpp>

class Gait{
public:
    virtual void getGaitTable() = 0;
    virtual void getChangeGaitTable() = 0;
};

class OffsetGait : public Gait {
public:
    OffsetGait(int mpcHorizon, uint64_t cyclePeriod, Vec4<int> offsets, Vec4<int> durations);
    ~OffsetGait();
    void getGaitTable() override;
    void getChangeGaitTable() override;

private:
    int mGaitTable[MPC_HORIZON*4];
    int mHorizon;
    int mCyclePeriod;
    Eigen::Array4i mOffsets;
    Eigen::Array4i mDurations;
};



#endif //RAISIM_GAIT_H
