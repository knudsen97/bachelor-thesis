/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#define PI 3.14159265
#define ANGLE_THRESHOLD 2*M_PI/32
#define IDLE_TIME 50

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
    //argos::Real Ku = 100000;
    //argos::Real Tu = 4.6;
    //con(SAMPLING_RATE*5, 0.6*Ku, 1.2*Ku/Tu, 0.10*Ku*Tu);

    argos::Real Ku = 20000;
    argos::Real Tu = 5;
    //con(SAMPLING_RATE*5, 0.75*Ku, 0, 0.12*Ku*Tu);
    //con(SAMPLING_RATE*5, 0.8*Ku, 0, 0.12*Ku*Tu);
    con(SAMPLING_RATE*5, 0.75*Ku, 0, 0.135*Ku*Tu);

}

    

void test_controller::ControlStep()
{
    /*Protocol receive variables*/
    argos::CVector3 goalPointMessage;
    argos::CRadians goalAngle, bugGoalAngle;
    argos::Real pushVelocity;


    /*Make controller instance*/
    controller::wVelocity wVel;
    argos::CVector3 goalPoint = {1, 1, 0};
    argos::CVector3 robotPos = posSensor->GetReading().Position;
    argos::CRadians robotAngle, temp;
    posSensor->GetReading().Orientation.ToEulerAngles(robotAngle, temp, temp);
    //std::cout << "Rob angle:  " << robotAngle << std::endl;

    bugGoalAngle = bugAlg.move(m_pcProximity, posSensor, goalPoint);
    //std::cout << "Goal angle: " << bugGoalAngle << std::endl;

    //std::cout << wVel.lWheel << " " << wVel.rWheel << std::endl;
    wVel = con.angleControl(robotAngle, bugGoalAngle);
    
    const argos::Real velScale = 1;
    wVel.lWheel *= velScale;
    wVel.rWheel *= velScale;

    //const argos::Real vel = 50;
   // m_pcWheels->SetLinearVelocity(100, -100);
    //std::cout << wVel.lWheel << wVel.rWheel << '\n';
    //m_pcWheels->SetLinearVelocity(wVel.lWheel, -wVel.rWheel);



    argos::Real leftWheeleVelocity;
    argos::Real rightWheeleVelocity;

    leftWheeleVelocity = wVel.lWheel;
    rightWheeleVelocity = wVel.rWheel;

    std::cout << "Diff:  " << abs(bugGoalAngle.GetValue() - robotAngle.GetValue()) << '\n';
    std::cout << "Thres: " << ANGLE_THRESHOLD << '\n';
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
        leftWheeleVelocity = wVel.lWheel + 60;
        rightWheeleVelocity = wVel.rWheel - 60;
        bugAlg.regulateSpeed(m_pcProximity, leftWheeleVelocity, rightWheeleVelocity);
        m_pcWheels->SetLinearVelocity(leftWheeleVelocity, rightWheeleVelocity);
    }

    // std::cout << "vel: " << leftWheeleVelocity <<  " " << rightWheeleVelocity  << std::endl;
}

REGISTER_CONTROLLER(test_controller, "test_controller")
