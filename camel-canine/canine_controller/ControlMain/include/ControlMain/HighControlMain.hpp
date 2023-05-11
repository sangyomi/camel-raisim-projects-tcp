//
// Created by hs on 23. 2. 9.
//

#ifndef RAISIM_HIGHCONTROLMAIN_HPP
#define RAISIM_HIGHCONTROLMAIN_HPP

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <convexMPC/MPCController.hpp>
#include <ControlUtils/Gait.hpp>

class HighControlMain
{
public:
    HighControlMain();

    void ControllerFunction();
private:
    uint64_t mGaitPeriod;
    uint64_t mIteration;

    bool bIsEndTrot;

    MPCController MPC;
    OffsetGait stand, trot;
};

#endif //RAISIM_HIGHCONTROLMAIN_HPP
