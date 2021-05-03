/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


#define PI 3.14159265
#define ANGLE_THRESHOLD 2*M_PI/32
#define IDLE_TIME 50

#define SAMPLING_RATE 0.01
#define PORT 1024

#define RECEIVE 0
#define GO_TO_POINT 1
#define UPDATE_SERVER 2
#define ORIENTATE 3
#define WAIT 4
#define SET_VELOCITY 5

#define PI 3.14159265
float scale = 0.01;

    test_controller::test_controller() :
        m_pcWheels(NULL),
        m_fWheelVelocity(2.5f),
        posSensor(NULL),
        pcBox(NULL){}

    void test_controller::Init(TConfigurationNode& t_node) 
    {
        
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        posSensor = GetSensor<CCI_PositioningSensor>("positioning");
        m_pcProximity = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
        GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    }

 

    void test_controller::ControlStep()
    {
    /*Protocol receive variables*/
    argos::CVector3 goalPointMessage;
    argos::CRadians goalAngle, bugGoalAngle;
    argos::Real pushVelocity;


    /*Make controller instance*/
    
    controller::wVelocity wVel;
    argos::CVector3 goalPoint = {2, 1.5, 0};
    argos::CVector3 robotPos = posSensor->GetReading().Position;
    argos::CRadians robotAngle, temp;
    posSensor->GetReading().Orientation.ToEulerAngles(temp, temp, robotAngle);
    

    bugGoalAngle = bugAlg.move(m_pcProximity, posSensor, goalPoint);
    con(SAMPLING_RATE*5, 2000, 100, 1);
    wVel = con.angleControl(robotAngle, bugGoalAngle);

    argos::Real leftWheeleVelocity;
    argos::Real rightWheeleVelocity;
    if(sqrt(pow(goalPoint.GetX() - robotPos.GetX(), 2) + pow(goalPoint.GetY() - robotPos.GetY(), 2)) <= 0.01999f)
    {
        m_pcWheels->SetLinearVelocity(0,0);
    }
    else if(abs(bugGoalAngle.GetValue() - robotAngle.GetValue()) >= ANGLE_THRESHOLD)
    {
        leftWheeleVelocity = wVel.lWheel;
        rightWheeleVelocity = wVel.rWheel;
        m_pcWheels->SetLinearVelocity(leftWheeleVelocity, -rightWheeleVelocity);
    }
    else
    {
        leftWheeleVelocity = wVel.lWheel + 4;
        rightWheeleVelocity = wVel.rWheel + 4;
        bugAlg.regulateSpeed(m_pcProximity, leftWheeleVelocity, rightWheeleVelocity);
        m_pcWheels->SetLinearVelocity(leftWheeleVelocity, -rightWheeleVelocity);
    }
    }

   REGISTER_CONTROLLER(test_controller, "test_controller")
