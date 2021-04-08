/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* For plotting */
#include "../inc/matplotlibcpp.h"

namespace plt = matplotlibcpp;


#define PI 3.14159265
#define ANGLE_THRESHOLD 2*M_PI/32

#define SAMPLING_RATE 0.01
#define PORT 1024

#define RECEIVE 0
#define GO_TO_POINT 1
#define UPDATE_SERVER 2
#define ORIENTATE 3
#define WAIT 4
#define SET_VELOCITY 5


int wait_time = 0;

void test_controller::connect()
{
    while (!this->clientSocket.IsConnected())
    {
        try
        {
            this->clientSocket.Connect("localhost", PORT);
        }
        catch(...)
        {
            ;
        }
    }
    
}

test_controller::test_controller() :
    m_pcWheels(NULL),
    m_fWheelVelocity(2.5f),
    posSensor(NULL),
    proxSensor(NULL){}

void test_controller::Init(TConfigurationNode& t_node) 
{

    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    posSensor = GetSensor<CCI_PositioningSensor>("positioning");
    proxSensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    connecting = std::thread{[=] { connect();}};

    currentState = RECEIVE;
}
void test_controller::ControlStep()
{
    //argos::LOG << "footbot id: "<< CCI_Controller::GetId() << '\n';
    const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();
    CRadians xAngle, yAngle, zAngle;
    robotPos.Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
    robotPosition = robotPos.Position; 

    if (!joined)
    {
        connecting.join();
        joined = true;
        connection(clientSocket);
    } 
    if(!sentPosition)
        sentPosition = connection.send(robotPosition);

    /************************* FSM START *************************/
    switch (currentState)
    {
    /************************* RECEIVE *************************/
    case RECEIVE:
    {
        wait_time++;
        std::cout << "CLIENT RECEIVE\n";

        if(connection.recieve())
        {
            switch (connection.getMessageType())
            {
            case protocol::dataType::typeCVector3:
                connection.getMessage(goalPointMessage);
                if(goalPointMessage != CVector3(0,0,0))
                {
                    currentState = GO_TO_POINT;
                }
                break;
            
            case protocol::dataType::typeCRadians:
                connection.getMessage(goalAngle);
                currentState = ORIENTATE;
                break;

            case protocol::dataType::typeReal:
                connection.getMessage(pushVelocity);
                currentState = SET_VELOCITY;
                break;

            default:
                break;
            }
        }
        if (wait_time > 20 && !sendWaitState)
        {
            connection.send(robotPos.Position);
            wait_time = 0;
        }
        else if(wait_time > 20 && sendWaitState)
        {
            sendWaitState = false;
            currentState = WAIT;
            wait_time = 0;
        }
        
    }
    break;

    /************************* GO TO POINT *************************/
    case GO_TO_POINT:
    {
        std::cout << "CLIENT GO_TO_POINT\n";

        argos::CRadians desiredAngle;
        desiredAngle = argos::ATan2(goalPointMessage.GetY() - robotPos.Position.GetY(), goalPointMessage.GetX() - robotPos.Position.GetX());
        pointReached = test_controller::ReadyToPush(robotPos, goalPointMessage, desiredAngle, xAngle);
       
        if(pointReached)
            currentState = UPDATE_SERVER;

    }
    break;

    /************************* UPDATE SERVER *************************/
    case UPDATE_SERVER:
    {
        std::cout << "CLIENT UPDATE SERVER\n";
        if(connection.send(robotPos.Position))
        {
            currentState = RECEIVE;
        }
        wait_time = 0;
    }
    break;

    /************************* ORIENTATE *************************/
    case ORIENTATE:
    {
        std::cout << "CLIENT ORIENTATE\n";

        controller con(SAMPLING_RATE*5, 2000, 100, 1);
        controller::wVelocity wVel;
        wVel = con.angleControl(xAngle, goalAngle);

        // std::cout << "ROBOT ORIEN: " << abs(goalAngle.GetValue()) - abs(xAngle.GetValue()) << std::endl;
        // std::cout << "ANGLE THRES: " << ANGLE_THRESHOLD << std::endl;
        if(abs(goalAngle.GetValue() - xAngle.GetValue()) >= 0.024999f)
        {
            m_pcWheels->SetLinearVelocity(wVel.lWheel, wVel.rWheel);
        }
        else
        {
            m_pcWheels->SetLinearVelocity(0,0);
            currentState = WAIT;
        }

    }
    break;

    /************************* WAIT *************************/
    case WAIT:
    {
        std::cout << "CLIENT WAIT\n";
        if(connection.send("WAIT"))
        {
            sendWaitState = true;
            currentState = RECEIVE;
        }


        break;
    }

    /************************* SET_VELOCITY *************************/
    case SET_VELOCITY:
    {
        std::cout << "CLIENT SET_VELOCITY\n";
        m_pcWheels->SetLinearVelocity(pushVelocity, pushVelocity);

        std::string message;
        if(connection.recieve(message))
        {
            std::cout << message << std::endl;
            if(message == "STOP")
                currentState = RECEIVE;
        }

        break;
    }


    }
}

/**
 * A function to make the robot go to an assigned corner position
 * @param robotPos The robot positioning sensor
 * @param goalPoint The goal point
 * @param desiredAngle The desired angle to reach goal
 * @param robotAngle The robots current angle
 */
bool test_controller::ReadyToPush(const CCI_PositioningSensor::SReading& robotPos, 
                                    argos::CVector3& goalPoint, argos::CRadians& desiredAngle, argos::CRadians& robotAngle, int v0)
{
    /*Make controller instance*/
    controller con(SAMPLING_RATE*5, 2000, 100, 1);
    controller::wVelocity wVel;

    bugGoalAngle = bugAlg.move(proxSensor, posSensor, goalPoint);
    wVel = con.angleControl(robotAngle, bugGoalAngle);

    argos::Real leftWheeleVelocity;
    argos::Real rightWheeleVelocity;
    if(sqrt(pow(goalPoint.GetX() - robotPos.Position.GetX(), 2) + pow(goalPoint.GetY() - robotPos.Position.GetY(), 2)) <= 0.01999f)
    {
        m_pcWheels->SetLinearVelocity(0,0);
        return true;
    }
    else if(abs(bugGoalAngle.GetValue() - robotAngle.GetValue()) >= ANGLE_THRESHOLD)
    {
        leftWheeleVelocity = wVel.lWheel;
        rightWheeleVelocity = wVel.rWheel;
        m_pcWheels->SetLinearVelocity(leftWheeleVelocity, rightWheeleVelocity);
    }
    else
    {
        leftWheeleVelocity = wVel.lWheel + v0;
        rightWheeleVelocity = wVel.rWheel + v0;
        bugAlg.regulateSpeed(proxSensor, leftWheeleVelocity, rightWheeleVelocity);
        m_pcWheels->SetLinearVelocity(leftWheeleVelocity, rightWheeleVelocity);
    }

    return false;
}

size_t test_controller::robotBufferSize = BUFFERSIZE;

REGISTER_CONTROLLER(test_controller, "test_controller")
