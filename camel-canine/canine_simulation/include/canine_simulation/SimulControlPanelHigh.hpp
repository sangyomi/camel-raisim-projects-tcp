//
// Created by hs on 23. 2. 9.
//

#ifndef RAISIM_SIMULCONTROLPANELHIGH_HPP
#define RAISIM_SIMULCONTROLPANELHIGH_HPP

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <convexMPC/MPCController.hpp>
#include <ControlUtils/Gait.hpp>

class SimulControlPanelHigh{
public:
    SimulControlPanelHigh();

    void ControllerFunction();
private:
    uint64_t mGaitPeriod;
    uint64_t mIteration;

    bool bIsEndTrot;

    MPCController MPC;
    OffsetGait stand, trot;
};

#endif //RAISIM_SIMULCONTROLPANELHIGH_HPP
