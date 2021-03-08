/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include "../inc/matplotlibcpp.h"

//#include "../matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;


#define PI 3.14159265
#define SAMPLING_RATE 0.01
#define V_0 4
#define PORT 1024

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


    plt::plot({ 1,3,2,4 });
    plt::show();

    connecting = std::thread{[=] { connect();}};
}


void test_controller::ControlStep()
{
    if (!joined)
    {
        connecting.join();
        joined = true;
    }

    if(clientSocket.GetEvents().find(argos::CTCPSocket::EEvent::InputReady) != clientSocket.GetEvents().end())
    {
        clientSocket.ReceiveByteArray(argosBuffer);
    }
    argos::LOG << "client recieved: " << argosBuffer << '\n';

    CVector3 goal;
    goal.Set(3.3, 3.3, 0);
    //std::vector<CVector3> validPushPoints = this->A.findPushPoints(pcBox, goal);
    std::vector<CVector3> validPushPoints;
    validPushPoints = P.FindPushPoints(pcBox,goal);

    if(!planComplete)
    {
        CVector3 goalLoc, startLoc;
        //Find robot position:
        CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("foot-bot");
        for (CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
        {
            CFootBotEntity* fBot = any_cast<CFootBotEntity*>(i->second);
            startLoc = fBot->GetEmbodiedEntity().GetOriginAnchor().Position;
        }
        goalLoc = validPushPoints[1];
        C.camera::GetPlot(this->map);
        this->map = P.planner::Wavefront(this->map,startLoc, goalLoc);
        P.planner::Pathfinder(map, startLoc, goalLoc);
        this->planComplete = true;
    }

    const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();
    CRadians xAngle, yAngle, zAngle;
    robotPos.Orientation.ToEulerAngles(xAngle, yAngle, zAngle);

    //Calculate the angle between robot position and goal position:
    argos::CRadians desiredAngle;
    desiredAngle = argos::ATan2(validPushPoints[1].GetY()-robotPos.Position.GetY(), validPushPoints[1].GetX() - robotPos.Position.GetX());

    //Make controller instance
    controller con(SAMPLING_RATE*5, 1000, 1, 1);

    controller::wVelocity wVel;
    wVel = con.angleControl(xAngle, desiredAngle);
    std::cout << wVel.lWheel << " " << wVel.rWheel << std::endl;

    if(abs(validPushPoints[1].GetX() - robotPos.Position.GetX()) <= 0.2f)
        m_pcWheels->SetLinearVelocity(0,0);
    else
        m_pcWheels->SetLinearVelocity(wVel.lWheel + V_0, wVel.rWheel + V_0);

    //adding a constant velocity otherwise the robot would just spin around it self.
    // if((velocities[0] > 0.499f || velocities[0] < -0.499f) && (velocities[1] > 0.499f || velocities[1] < -0.499f))
    //     m_pcWheels->SetLinearVelocity(velocities[0], velocities[1]);
    // else
    //     m_pcWheels->SetLinearVelocity(V_0, V_0);

}

size_t test_controller::robotBufferSize = BUFFERSIZE;

REGISTER_CONTROLLER(test_controller, "test_controller")
