/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* TCP communication */
#include <argos3/core/utility/networking/tcp_socket.h>
#include "../inc/matplotlibcpp.h"

//#include "../matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;


#define PI 3.14159265
#define ANGLE_THRESHOLD 2*M_PI/32

#define SAMPLING_RATE 0.01
#define V_0 4

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
    pcBox = new CBoxEntity("box1",                   // id
                            CVector3(1, 1.7, 0.0), // position
                            CQuaternion(),           // orientation
                            true,                    // movable or not?
                            CVector3(0.1, 0.4, 0.5), // size
                            500.0);                    // mass in kg
    AddEntity(*pcBox);

    plt::plot({ 1,3,2,4 });
    plt::show();

}


void test_controller::ControlStep()
{
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
    argos::CVector3 goalPoint = validPushPoints[0];
    argos::CRadians desiredAngle;
    desiredAngle = argos::ATan2(goalPoint.GetY()-robotPos.Position.GetY(), goalPoint.GetX() - robotPos.Position.GetX());

    //Make controller instance
    controller con(SAMPLING_RATE*5, 1000, 1, 1);

    controller::wVelocity wVel;
    wVel = con.angleControl(xAngle, desiredAngle);
    std::cout << wVel.lWheel << " " << wVel.rWheel << std::endl;
 
    if(abs(goalPoint.GetX() - robotPos.Position.GetX()) <= 0.1999f)
        m_pcWheels->SetLinearVelocity(0,0);
    else if(abs(desiredAngle.GetValue()) - abs(xAngle.GetValue()) >= ANGLE_THRESHOLD)
    {
        m_pcWheels->SetLinearVelocity(wVel.lWheel, wVel.rWheel);
    }
    else
        m_pcWheels->SetLinearVelocity(wVel.lWheel + V_0, wVel.rWheel + V_0);

}

REGISTER_CONTROLLER(test_controller, "test_controller")
