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
 
#define PLANNING        0
#define READY_TO_PUSH   1
#define WAIT            2
#define PUSH            3

bool joined = false;

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
    pcBox(NULL),
    proxSensor(NULL){}
    //camSensor(NULL){}

void test_controller::Init(TConfigurationNode& t_node) 
{

    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    posSensor = GetSensor<CCI_PositioningSensor>("positioning");
    proxSensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    //camSensor = GetSensor<CCI_CameraSensor>("camera0");

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
    for (CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator)
    {   
        pcBox = any_cast<CBoxEntity*>(iterator->second);
        if (pcBox->GetId() == "box1")
            break;
    }

    connecting = std::thread{[=] { connect();}};

    currentState = PLANNING;
}
void test_controller::ControlStep()
{
    if (!joined)
    {
        connecting.join();
        joined = true;
        connection(clientSocket);
    }

    if(clientSocket.GetEvents().find(argos::CTCPSocket::EEvent::InputReady) != clientSocket.GetEvents().end())
    {
        clientSocket.ReceiveByteArray(argosBuffer);
    }
    //argos::LOG << "client recieved: " << argosBuffer << '\n';
    argos::LOG << "bool recieved: " << connection.recieve(argosBuffer) << '\n';
    argos::LOG << "client recieved: " << argosBuffer << '\n';

    //Location where box needs to go:
    CVector3 goal;

    goal.Set(2, 2, 0);
    const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();
    CRadians xAngle, yAngle, zAngle;
    robotPos.Orientation.ToEulerAngles(xAngle, yAngle, zAngle);

    /************************* FSM START *************************/
    switch (currentState)
    {
    /************************* PLANNING *************************/
    case PLANNING:
    {
        planComplete = Planning(goal);
        try
        {
            if(planComplete)
            {
                currentState = READY_TO_PUSH;
                std::cout << "PlANNING COMPLETE\n";
            }
            else
                throw(planComplete);
        }
        catch(bool e)
        {
            std::cerr << "An exception occured. Planning state: " << e << std::endl;
            LOGERR << "An exception occured. Planning state: " << e << std::endl;
        }
    }
    break;

    /************************* READT TO PUSH *************************/
    case READY_TO_PUSH:
    {
        //Calculate the angle between robot position and goal position:
        argos::CVector3 goalPoint = argos::CVector3(subGoals[i].x/(double)SCALE, subGoals[i].y/(double)SCALE, 0);
        //std::cout << goalPoint << std::endl;

        argos::CRadians desiredAngle;
        desiredAngle = argos::ATan2(goalPoint.GetY()-robotPos.Position.GetY(), goalPoint.GetX() - robotPos.Position.GetX());

        bool readyToPush = false;
        readyToPush = test_controller::ReadyToPush(robotPos, goalPoint, desiredAngle, xAngle);
        if(readyToPush)
        {
            currentState = WAIT;
            std::cout << "READY TO PUSH COMPLETE\n";
        }
    }
    break;
    
    /************************* WAIT *************************/
    case WAIT:
    {
        argos::CRadians goalAngle;
        goalAngle = argos::ATan2(goal.GetY()-robotPos.Position.GetY(), goal.GetX() - robotPos.Position.GetX());

        controller con(SAMPLING_RATE*5, 2000, 100, 1);

        controller::wVelocity wVel;
        wVel = con.angleControl(xAngle, goalAngle);

        if(abs(goalAngle.GetValue()) - abs(xAngle.GetValue()) >= ANGLE_THRESHOLD)
        {
            m_pcWheels->SetLinearVelocity(wVel.lWheel, wVel.rWheel);
        }
        else
        {
            m_pcWheels->SetLinearVelocity(wVel.lWheel, wVel.rWheel);
        }

        //m_pcWheels->SetLinearVelocity(0,0);
        //Some code to check if all the other robots also have reached their corner.
        //When that is true:
        //currenState = PUSH
    }
    break;

    case PUSH:
    {

    }
    break;

    }
}

/**
 * A function to call the planning algorithms such as wavefront and pathfinder
 * @param goal The goal position
*/
bool test_controller::Planning(argos::CVector3 &goal)
{
    //Find where to push on the box to get to goal:
    std::vector<CVector3> validPushPoints;
    validPushPoints = P.FindPushPoints(pcBox, goal);

    CVector3 goalLoc, startLoc;
    //Find robot position:
    CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("foot-bot");
    for (CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
    {
        CFootBotEntity* fBot = any_cast<CFootBotEntity*>(i->second);
        startLoc = fBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    }

    //Assign a corner to do wavefront/pathfinder on:
    goalLoc = validPushPoints[0];
    C.camera::GetPlot(this->map);

    //Find a point on the line between corner and goal
    argos::CVector3 CG = goal - goalLoc; //Corner-Goal vector
    CG = CG.Normalize() * (-OFF_SET);
    goalLoc += CG;
    //Den tegner det godt nok, men robotten kommer ikke derhen, hvilket jeg tror er pga. pathfinderen

    this->map = P.planner::Wavefront(this->map,startLoc, goalLoc);
    subGoals = P.planner::Pathfinder(map, startLoc, goalLoc);

    return true;
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
        i++;
        //std::cout << "i: " << this->i << std::endl;
        m_pcWheels->SetLinearVelocity(0,0);
        if(i >= subGoals.size())
            cornerFound = true;
    }
    else if(cornerFound)
    {
        m_pcWheels->SetLinearVelocity(0,0);
        //std::cout << "CORNER FOUND\n";
        return true;
        // plt::plot(con.getX(), "r--");
        // plt::plot(con.getY());
        // plt::show();
    }
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


// void test_controller::ControlStep()
// {
//     if (!joined)
//     {
//         connecting.join();
//         joined = true;
//     }

//     if(clientSocket.GetEvents().find(argos::CTCPSocket::EEvent::InputReady) != clientSocket.GetEvents().end())
//     {
//         clientSocket.ReceiveByteArray(argosBuffer);
//     }
//     //argos::LOG << "client recieved: " << argosBuffer << '\n';

//     //Location where box needs to go:
//     CVector3 goal;
//     goal.Set(3.3, 3.3, 0);

//     // switch (currentState)
//     // {
//     // case PLANNING:
//     //     try
//     //     {
//     //         if(!planComplete)
//     //         {
//     //             planComplete = Planning(goal);
//     //             currentState = READY_TO_PUSH;
//     //         }
//     //     }
//     //     catch(const std::exception& e)
//     //     {
//     //         std::cerr << "Planning not complete" << '\n';
//     //     }
        
//     //     break;
    
//     // default:
//     //     break;
//     // }
//     //First time do path planning
//     if(!planComplete)
//         planComplete = Planning(goal);
 
//     const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();
//     CRadians xAngle, yAngle, zAngle;
//     robotPos.Orientation.ToEulerAngles(xAngle, yAngle, zAngle);

//     //Calculate the angle between robot position and goal position:
//     argos::CVector3 goalPoint = argos::CVector3(subGoals[i].x/(double)SCALE, subGoals[i].y/(double)SCALE, 0);
//     std::cout << goalPoint << std::endl;

//     argos::CRadians desiredAngle;
//     desiredAngle = argos::ATan2(goalPoint.GetY()-robotPos.Position.GetY(), goalPoint.GetX() - robotPos.Position.GetX());

//     //Make controller instance
//     test_controller::ReadyToPush(robotPos, goalPoint, desiredAngle, xAngle);
// }
