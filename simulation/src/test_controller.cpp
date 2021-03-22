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
 
// #define PLANNING        0
// #define READY_TO_PUSH   1
// #define WAIT            2
// #define PUSH            3

#define RECEIVE 0
#define GO_TO_POINT 1
#define UPDATE_SERVER 2
#define ORIENTATE 3
#define WAIT 4
#define PUSH_TO_GOAL 5

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
    //pcBox(NULL),
    proxSensor(NULL){}
    //camSensor(NULL){}

void test_controller::Init(TConfigurationNode& t_node) 
{

    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    posSensor = GetSensor<CCI_PositioningSensor>("positioning");
    proxSensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    //camSensor = GetSensor<CCI_CameraSensor>("camera0");

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    // CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
    // for (CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator)
    // {   
    //     pcBox = any_cast<CBoxEntity*>(iterator->second);
    //     if (pcBox->GetId() == "box1")
    //         break;
    // }

    connecting = std::thread{[=] { connect();}};

    currentState = RECEIVE;
}
void test_controller::ControlStep()
{
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
    argos::LOG << "sentPosition: " << sentPosition << '\n';


    //std::cout << "waittime: " << wait_time << '\n';
    /************************* FSM START *************************/
    switch (currentState)
    {
    /************************* RECEIVE *************************/
    case RECEIVE:
    {
        wait_time++;
        std::cout << "CLIENT RECEIVE\n";
        //protocol::dataType::
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

            default:
                break;
            }
        }
        if (wait_time > 20)
        {
            connection.send(robotPos.Position);
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
    }
    break;

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
    //Make controller instance
    controller con(SAMPLING_RATE*5, 2000, 100, 1);
    controller::wVelocity wVel;
    wVel = con.angleControl(robotAngle, desiredAngle);
    //std::cout << wVel.lWheel << " " << wVel.rWheel << std::endl;
    
    if(abs(goalPoint.GetX() - robotPos.Position.GetX()) <= 0.01999f)
    {
        //i++;
        //std::cout << "i: " << this->i << std::endl;
        m_pcWheels->SetLinearVelocity(0,0);
        //pointReached = true;
        //std::cout << "test\n";
        // if(i >= subGoals.size())
        //     cornerFound = true;
        return true;
    }
    // else if(cornerFound)
    // {
    //     m_pcWheels->SetLinearVelocity(0,0);
    //     //std::cout << "CORNER FOUND\n";
    //     //std::cout << "hej\n";

    //     return true;
    //     // plt::plot(con.getX(), "r--");
    //     // plt::plot(con.getY());
    //     // plt::show();
    // }
    else if(abs(desiredAngle.GetValue()) - abs(robotAngle.GetValue()) >= ANGLE_THRESHOLD)
    {
        m_pcWheels->SetLinearVelocity(wVel.lWheel, wVel.rWheel);
    }
    else
    {
        m_pcWheels->SetLinearVelocity(wVel.lWheel + v0, wVel.rWheel + v0);
    }

    return false;
}

size_t test_controller::robotBufferSize = BUFFERSIZE;

REGISTER_CONTROLLER(test_controller, "test_controller")
