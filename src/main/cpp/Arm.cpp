#include "Arm.h"

// Constructor
Arm::Arm()
{
    m_eState = arm::eState::STATE_START;
    m_eCommand = arm::COMMAND_HOME;

    m_pRobotIO = nullptr;
}

void Arm::Initialize(RobotIO *p_pRobotIO)
{
    m_pRobotIO = p_pRobotIO;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();
}

void Arm::UpdateInputStatus()
{

}

void Arm::Execute()
{
    // Check if m_pRobotIO has been assigned
    if(m_pRobotIO != nullptr)
    {
        // ***************
        // * Start State *
        // ***************
        if(m_eState == arm::eState::STATE_START)
        {
            m_eState = arm::eState::STATE_IDLE;
        }
        // **************
        // * Idle State *
        // **************
        if(m_eState == arm::eState::STATE_IDLE)
        {
            // *--------------*
            // * Home Command *
            // *--------------*

            if(m_eCommand == arm::COMMAND_NONE)
            {
                if(/*At lower Limit*/)
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply Motor configs

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed

                //Transition to homing state
                m_eState = arm::eState::STATE_HOMING;
            }

            // *----------------------*
            // * Manual Raise Command *
            // *----------------------*

            else if(m_eCommand == arm::COMMAND_MANUAL_RAISE)
            {

                //Apply Motor configs

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed

                //Transition to Manual raise state
                m_eState = arm::eState::STATE_MANUAL_RAISE;
            }

            // *----------------------*
            // * Manual Lower Command *
            // *----------------------*

            else if(m_eCommand == arm::COMMAND_MANUAL_LOWER)
            {
                if(/*At lower Limit*/)
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply Motor configs

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed

                //Transition to Manual lower state
                m_eState = arm::eState::STATE_MANUAL_LOWER;
            }

            // *--------------------*
            // * Auto Raise Command *
            // *--------------------*

            else if(m_eCommand == arm::COMMAND_AUTO_RAISE)
            {
                if(/*Above encoder setpoint*/)
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply Motor configs

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed

                //Transition to auto raise state
                m_eState = arm::eState::STATE_AUTO_RAISE;
            }

            // Error - unrecognized command

            else if(m_eCommand != arm::COMMAND_NONE && m_eCommand != arm::COMMAND_STOP)
            {
                printf("Error in Arm.cpp - unknown command \n");
            }
        }
        // ****************
        // * Homing State *
        // ****************
        else if(m_eState == arm::eState::STATE_HOMING)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dHomingTimeout)
            {
                bIsTimedOut = true;
            }

            if(/*Check Arm Limit Switch */ || bIsTimedOut)
            {
                // Stop Motors
                //Enable brake mode
                // Reset Encoders

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        // **********************
        // * Manual Raise State *
        // **********************
        else if(m_eState == arm::eState::STATE_MANUAL_RAISE)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dManualRaiseTimeout)
            {
                bIsTimedOut = true;
            }
            if(m_eCommand == arm::COMMAND_STOP || bIsTimedOut)
            {
                // Stop Motors
                //Enable brake mode

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        // **********************
        // * Manual Lower State *
        // **********************
        else if(m_eState == arm::eState::STATE_MANUAL_LOWER)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dManualLowerTimeout)
            {
                bIsTimedOut = true;
            }
            if(m_eCommand == arm::COMMAND_STOP || /*Check Arm Limit Switch */ || bIsTimedOut)
            {
                // Stop Motors
                //Enable brake mode

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        // ********************
        // * Auto Raise State *
        // ********************
        else if(m_eState == arm::eState::STATE_AUTO_RAISE)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dAutoRaiseTimeout)
            {
                bIsTimedOut = true;
            }
            if(/*Check Motor encoder agains setpoint*/ || bIsTimedOut)
            {
                // Stop Motors
                //Enable brake mode

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }

        // Handle Error State or unknown state
        else
        {
            //error logic here if needed
            printf("Error in Arm.cpp - Error or unknown state\n");
        }
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("ERROR in Arm.cpp - Robot IO Pointer Null\n");
    }
}