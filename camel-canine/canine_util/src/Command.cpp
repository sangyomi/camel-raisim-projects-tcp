#include <canine_util/Command.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

Command::Command()
: mLocalVelocity(0.0)
, mDeadBand(2000)
{
    initializeJoystick();
}

void Command::commandFunction()
{
    readJoystick();
    mappingDSJoystickCommand();
//    mappingXBOXJoystickCommand();
    if(sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch(incomingCommand)
        {
            case CAN_ON:
            {
                sharedMemory->canLFState = CAN_INIT;
                sharedMemory->canRFState = CAN_INIT;
                sharedMemory->canLBState = CAN_INIT;
                sharedMemory->canRBState = CAN_INIT;

                sharedMemory->newCommand = true;
                sharedCommand->userCommand = VISUAL_ON;
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                sharedMemory->newCommand = true;
                sharedCommand->userCommand = MOTOR_ON;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->canLFState = CAN_MOTOR_ON;
                sharedMemory->canRFState = CAN_MOTOR_ON;
                sharedMemory->canLBState = CAN_MOTOR_ON;
                sharedMemory->canRBState = CAN_MOTOR_ON;
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->canLFState = CAN_MOTOR_OFF;
                sharedMemory->canRFState = CAN_MOTOR_OFF;
                sharedMemory->canLBState = CAN_MOTOR_OFF;
                sharedMemory->canRBState = CAN_MOTOR_OFF;
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
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            case READY:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_DOWN_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_DOWN_READY;
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            case CUSTOM_1:
            {
                break;
            }
            case CUSTOM_2:
            {
                for(int i = 0 ; i < MOTOR_NUM ; i++)
                {
                    sharedMemory->motorDesiredTorque[i] = 0.0;
                }
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            default:
                break;
        }
    }
}

void Command::initializeJoystick()
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

void Command::readJoystick()
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
    mLocalVelocity = -(double)mJoystickAxis[1] / 100000.0;
    sharedMemory->baseDesiredVelocity[0] = mLocalVelocity * cos(sharedMemory->baseEulerPosition[2]);
    sharedMemory->baseDesiredVelocity[1] = mLocalVelocity * sin(sharedMemory->baseEulerPosition[2]);
    sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
}

void Command::printJoystickValue()
{
    std::cout << "axis/10000: ";
    for (size_t i(0); i < mJoystickAxis.size(); ++i)
    {
        std::cout << " " << std::setw(2) << (double)mJoystickAxis[i] / 10000;
    }
    std::cout << "  button: ";
    for (size_t i(0); i < mJoystickButton.size(); ++i)
    {
        std::cout << " " << (double)mJoystickButton[i];
    }
    std::cout << std::endl;
}

void Command::mappingDSJoystickCommand()
{
    switch(sharedMemory->FSMState)
    {
    case FSM_INITIAL:
    {
        if(mJoystickButton[9] == 1)
        {
            std::cout<<"[FSM] : INITIALIZE"<<std::endl;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CAN_ON;
            sharedMemory->FSMState = FSM_READY;
        }
        break;
    }
    case FSM_VISUAL_CAN_READY:
    {
        /// escape motor off
        std::cout<<"[FSM_VISUAL_CAN_READY]"<<std::endl;
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_READY:
    {
        std::cout<<"[FSM_READY]"<<std::endl;
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
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_ISOLATION:
    {
        std::cout<<"[FSM_ISOLATION]"<<std::endl;
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
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_CONST_STAND:
    {
        std::cout<<"[FSM_CONST_STAND]"<<std::endl;
        /// sit down
        if(mJoystickButton[7] == 1 && mJoystickAxis[7] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = READY;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// go FSM_TROT
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->FSMState = FSM_TROT;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_STAND:
    {
        std::cout<<"[FSM_STAND]"<<std::endl;
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
/*
        /// x, y, yaw axis trot
        if((abs(mLocalVelocity) > 0.04)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.04))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
*/
   /// trot change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_TROT:
    {
        std::cout<<"[FSM_TROT]"<<std::endl;
/*
        /// stand
        if((sqrt(pow(sharedMemory->baseVelocity[0],2)+pow(sharedMemory->baseVelocity[1],2)) < 0.025) && abs(sharedMemory->baseEulerVelocity[2]) < 0.04
           && !((abs(mLocalVelocity) > 0.00004)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.000004)))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
/*
   /// stand change button
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
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    default:
    {
        std::cout<<"[FSM_default]"<<std::endl;
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }

    }

}

void Command::mappingXBOXJoystickCommand()
{
    switch(sharedMemory->FSMState)
    {
    case FSM_INITIAL:
    {
        if(mJoystickButton[7] == 1)
        {
            std::cout<<"[FSM] : INITIALIZE"<<std::endl;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CAN_ON;
            sharedMemory->FSMState = FSM_READY;
        }
        break;
    }
    case FSM_VISUAL_CAN_READY:
    {
        /// escape motor off
        std::cout<<"[FSM_VISUAL_CAN_READY]"<<std::endl;
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_READY:
    {
        std::cout<<"[FSM_READY]"<<std::endl;
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
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_ISOLATION:
    {
        std::cout<<"[FSM_ISOLATION]"<<std::endl;
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
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;///////
    }
    case FSM_CONST_STAND:
    {
        std::cout<<"[FSM_CONST_STAND]"<<std::endl;
        /// sit down
        if(mJoystickAxis[5] > 0 && mJoystickAxis[7] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = READY;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// go FSM_TROT
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->FSMState = FSM_TROT;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
        }
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_STAND:
    {
        std::cout<<"[FSM_STAND]"<<std::endl;
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
/*
        /// x, y, yaw axis trot
        if((abs(mLocalVelocity) > 0.04)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.04))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
*/
        /// trot change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_TROT:
    {
        std::cout<<"[FSM_TROT]"<<std::endl;
/*
        /// stand
        if((sqrt(pow(sharedMemory->baseVelocity[0],2)+pow(sharedMemory->baseVelocity[1],2)) < 0.025) && abs(sharedMemory->baseEulerVelocity[2]) < 0.04
           && !((abs(mLocalVelocity) > 0.00004)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.000004)))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// go FSM_CONST_STAND
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
        }
/*
   /// stand change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    default:
    {
        std::cout<<"[FSM_default]"<<std::endl;
        /// motor off
        if(mJoystickAxis[5] > 0 && mJoystickAxis[2] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }

    }

}