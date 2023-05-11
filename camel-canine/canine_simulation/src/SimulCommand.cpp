//
// Created by hs on 22. 10. 28.
//

#include <canine_simulation/SimulCommand.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulCommand::SimulCommand()
: mLocalVelocity(0.0)
, mDeadBand(2000)
{
    initializeJoystick();
}

void SimulCommand::commandFunction()
{
    readJoystick();
    printJoystickValue();
//    mappingDSJoystickCommand();
    mappingXBOXJoystickCommand();
    if(sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch(incomingCommand)
        {
            case SIMUL_ON:
            {
                sharedMemory->visualState = STATE_UPDATE_VISUAL;
                break;
            }
            case SIMUL_OFF:
            {
                sharedMemory->visualState = STATE_VISUAL_STOP;
                break;
            }
            case CHANGE_GAIT_STAND:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_STAND;
                break;
            }
            case CHANGE_GAIT_TROT:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_TROT;
                break;
            }
            case CHANGE_GAIT_CUSTOM1:
            {
                sharedMemory->throwFlag = true;
                break;
            }
            case CHANGE_GAIT_CUSTOM2:
            {
                break;
            }
            case HOME:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_UP_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_UP_READY;
                break;
            }
            case READY:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_DOWN_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_DOWN_READY;
                break;
            }
            case CUSTOM_1:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->LowControlState = STATE_LOW_BACK_FLIP_READY;
                break;
            }
            case CUSTOM_2:
            {
                break;
            }
            default:
            {
                break;
            }
        }
    }
}

void SimulCommand::initializeJoystick()
{
    const char* joyStickFilePath = "/dev/input/js0";
    mJoystickFd = -1;
    mJoystickNumOfAxis = 0;
    mJoystickNumOfButton = 0;
    if ((mJoystickFd = open(joyStickFilePath, O_RDONLY)) < 0)
    {
        std::cerr << "Failed to open " << joyStickFilePath << std::endl;
        return;
    }
    ioctl(mJoystickFd, JSIOCGAXES, &mJoystickNumOfAxis);
    ioctl(mJoystickFd, JSIOCGBUTTONS, &mJoystickNumOfButton);
    ioctl(mJoystickFd, JSIOCGNAME(80), &mJoystickName);
    mJoystickButton.resize(mJoystickNumOfButton, 0);
    mJoystickAxis.resize(mJoystickNumOfAxis, 0);
    std::cout << "Joystick: " << mJoystickName << std::endl
              << "  axis: " << mJoystickNumOfAxis << std::endl
              << "  buttons: " << mJoystickNumOfButton << std::endl;
    fcntl(mJoystickFd, F_SETFL, O_NONBLOCK);   // using non-blocking mode
}

void SimulCommand::readJoystick()
{
    js_event js;
    read(mJoystickFd, &js, sizeof(js_event));
    switch (js.type & ~JS_EVENT_INIT)
    {
        case JS_EVENT_AXIS:
            if(((int)js.value > mDeadBand)||((int)js.value < -mDeadBand))
            {
                int sign;
                if((int)js.value > 0)
                {
                    sign = 1;
                }
                else
                {
                    sign = -1;
                }
                mJoystickAxis[(int)js.number] = js.value - sign * mDeadBand;
            }
            else
            {
                mJoystickAxis[(int)js.number] = 0;
            }
            break;
        case JS_EVENT_BUTTON:
            mJoystickButton[(int)js.number] = js.value;
            break;
    }

}

void SimulCommand::printJoystickValue()
{
    std::cout << "axis/10000: ";
    for (size_t i(0); i < mJoystickAxis.size(); ++i)
    {
        std::cout << " " << std::setw(2) << (double)mJoystickAxis[i];
    }
    std::cout << "  button: ";
    for (size_t i(0); i < mJoystickButton.size(); ++i)
    {
        std::cout << " " << (double)mJoystickButton[i];
    }
    std::cout << std::endl;
}

void SimulCommand::mappingDSJoystickCommand()
{
    switch(sharedMemory->FSMState)
    {
    case FSM_INITIAL:
    {
//        std::cout<<"[FSM_INITIAL]"<<std::endl;
        /// simul start
        if(mJoystickButton[9] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_ON;
            sharedMemory->FSMState = FSM_READY;
        }
        break;
    }
    case FSM_VISUAL_CAN_READY:
    {
//        std::cout<<"[FSM_VISUAL_CAN_READY]"<<std::endl;
        /// motor off == simul off
        break;
    }
    case FSM_READY:
    {
//        std::cout<<"[FSM_READY]"<<std::endl;
        /// stand up
        if(mJoystickButton[7] == 1 && mJoystickAxis[7] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = HOME;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_ISOLATION:
    {
//        std::cout<<"[FSM_ISOLATION]"<<std::endl;
        /// if end the sit down, go FSM_READY
        if(sharedMemory->bIsEndHome == true && sharedCommand->userCommand == READY)
        {
            sharedMemory->FSMState = FSM_READY;
            sharedMemory->bIsEndHome = false;
        }
        /// if end the stand up, go FSM_CONST_STAND
        if(sharedMemory->bIsEndHome == true && sharedCommand->userCommand == HOME)
        {
            std::cout<<"bISEndHome"<<sharedMemory->bIsEndHome<<std::endl;
            sharedMemory->FSMState = FSM_CONST_STAND;
            sharedMemory->bIsEndHome = false;
        }
        break;
    }
    case FSM_CONST_STAND:
    {
//        std::cout<<"[FSM_CONST_STAND]"<<std::endl;
        /// sit down
        if(mJoystickButton[7] == 1 && mJoystickAxis[7] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = READY;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// go FSM_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->FSMState = FSM_STAND;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        sharedMemory->baseDesiredEulerVelocity[0] = -(double)mJoystickAxis[1] / 75000.0;
        sharedMemory->baseDesiredEulerVelocity[1] = (double)mJoystickAxis[4] / 75000.0;
        sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
        break;
    }
    case FSM_STAND:
    {
//        std::cout<<"[FSM_STAND]"<<std::endl;
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
        /// x, y, yaw axis trot
        if((abs(mLocalVelocity) > 0.04)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.04))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        mLocalVelocity = -(double)mJoystickAxis[1] / 100000.0;
        sharedMemory->baseDesiredVelocity[0] = mLocalVelocity * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = mLocalVelocity * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
        break;
    }
    case FSM_TROT:
    {
//        std::cout<<"[FSM_TROT]"<<std::endl;
        /// stand
        if((sqrt(pow(sharedMemory->baseVelocity[0],2)+pow(sharedMemory->baseVelocity[1],2)) < 0.025) && abs(sharedMemory->baseEulerVelocity[2]) < 0.04
                    && !((abs(mLocalVelocity) > 0.00004)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.000004)))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
/*
 *  /// stand change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        mLocalVelocity = -(double)mJoystickAxis[1] / 100000.0;
        sharedMemory->baseDesiredVelocity[0] = mLocalVelocity * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = mLocalVelocity * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
        break;
    }
    default:
    {
        break;
    }

    }
}

void SimulCommand::mappingXBOXJoystickCommand()
{
    switch(sharedMemory->FSMState)
    {
    case FSM_INITIAL:
    {
//        std::cout<<"[FSM_INITIAL]"<<std::endl;
        /// simul start
        if(mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_ON;
            sharedMemory->FSMState = FSM_READY;
        }
        break;
    }
    case FSM_VISUAL_CAN_READY:
    {
//        std::cout<<"[FSM_VISUAL_CAN_READY]"<<std::endl;
        /// motor off == simul off
        break;
    }
    case FSM_READY:
    {
//        std::cout<<"[FSM_READY]"<<std::endl;
        /// stand up
        if(mJoystickAxis[5] > 0 && mJoystickAxis[7] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = HOME;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_ISOLATION:
    {
//        std::cout<<"[FSM_ISOLATION]"<<std::endl;
        /// if end the sit down, go FSM_READY
        if(sharedMemory->bIsEndHome == true && sharedCommand->userCommand == READY)
        {
            sharedMemory->FSMState = FSM_READY;
            sharedMemory->bIsEndHome = false;
        }
        /// if end the stand up, go FSM_CONST_STAND
        if(sharedMemory->bIsEndHome == true && sharedCommand->userCommand == HOME)
        {
            std::cout<<"bISEndHome"<<sharedMemory->bIsEndHome<<std::endl;
            sharedMemory->FSMState = FSM_CONST_STAND;
            sharedMemory->bIsEndHome = false;
        }
        break;
    }
    case FSM_CONST_STAND:
    {
//        std::cout<<"[FSM_CONST_STAND]"<<std::endl;
        /// sit down
        if(mJoystickAxis[5] > 0 && mJoystickAxis[7] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = READY;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// go FSM_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->FSMState = FSM_STAND;
        }
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        sharedMemory->baseDesiredEulerVelocity[0] = -(double)mJoystickAxis[1] / 50000.0;
        sharedMemory->baseDesiredEulerVelocity[1] = (double)mJoystickAxis[4] / 50000.0;
        sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 50000.0;
        break;
    }
    case FSM_STAND:
    {
//        std::cout<<"[FSM_STAND]"<<std::endl;
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
        /// x, y, yaw axis trot
        if((abs(mLocalVelocity) > 0.04)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.04))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        mLocalVelocity = -(double)mJoystickAxis[1] / 100000.0;
        sharedMemory->baseDesiredVelocity[0] = mLocalVelocity * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = mLocalVelocity * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
        break;
    }
    case FSM_TROT:
    {
//        std::cout<<"[FSM_TROT]"<<std::endl;
        /// stand
        if((sqrt(pow(sharedMemory->baseVelocity[0],2)+pow(sharedMemory->baseVelocity[1],2)) < 0.025) && abs(sharedMemory->baseEulerVelocity[2]) < 0.04
            && !((abs(mLocalVelocity) > 0.00004)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.000004)))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
/*
 *  /// stand change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        mLocalVelocity = -(double)mJoystickAxis[1] / 100000.0;
        sharedMemory->baseDesiredVelocity[0] = mLocalVelocity * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = mLocalVelocity * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
        break;
    }
    default:
    {
        break;
    }

    }
}