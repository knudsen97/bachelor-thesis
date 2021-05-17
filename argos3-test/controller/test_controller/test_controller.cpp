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
    pcBox(NULL),
    obj(NULL){}

void test_controller::Init(TConfigurationNode& t_node) 
{
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    posSensor = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcProximity = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    //argos::Real Ku = 100000;
    //argos::Real Tu = 4.6;
    //con(SAMPLING_RATE*5, 0.6*Ku, 1.2*Ku/Tu, 0.10*Ku*Tu);

    // argos::Real Ku = 20000;
    // argos::Real Tu = 5;
    // con(SAMPLING_RATE*5, 0.75*Ku, 0, 0.135*Ku*Tu);

    argos::Real Ku = 5000;
    argos::Real Tu = 5;
    con(SAMPLING_RATE*5, 2000, 0, 0);//0.135*Ku*Tu);
    // currentLocation = posSensor->GetReading().Position;
    // prevLocation = {0,0,0};
    out.open("out.csv");
}

argos::Real test_controller::getVelocity(const argos::CVector3 &cur, const argos::CVector3 &prev, argos::Real dt)
{
    argos::LOG << "diff: " << argos::CVector2(cur.GetX(), cur.GetY()) - argos::CVector2(prev.GetX(), prev.GetY()) << std::endl;
    return Distance(argos::CVector2(cur.GetX(), cur.GetY()), argos::CVector2(prev.GetX(), prev.GetY()))/(dt);

    // return abs(cur.GetX() - prev.GetX())/dt;

}
    

void test_controller::ControlStep()
{
    if(time == 0)
    {
        CSpace::TMapPerType& objMap = GetSpace().GetEntitiesByType("prototype");
        for (CSpace::TMapPerType::iterator i = objMap.begin(); i != objMap.end(); ++i)
        {
            obj = argos::any_cast<CPrototypeEntity*>(i->second);
            // std::string objId = obj->GetId();
        }
    }
    argos::CVector3 currentLocation = obj->GetEmbodiedEntity().GetOriginAnchor().Position;

    if(!testComplete)
        time++;

    /*Protocol receive variables*/
    argos::CVector3 goalPointMessage;
    argos::CRadians goalAngle, bugGoalAngle;
    argos::Real pushVelocity;

    /*Make controller instance*/
    controller::wVelocity wVel;
    argos::CVector3 goalPoint = {2, 1.5, 0};
    argos::CVector3 robotPos = posSensor->GetReading().Position;
    argos::CRadians robotAngle, temp;
    posSensor->GetReading().Orientation.ToEulerAngles(robotAngle, temp, temp);

    wVel = con.angleControl(robotAngle, bugGoalAngle);
    const argos::Real vel = 50;
    m_pcWheels->SetLinearVelocity(vel + wVel.lWheel, -vel + wVel.rWheel);


    if(time > 1)//time % 2 == 0 && time > 1)
    {
        // argos::Real dist = getVelocity(currentLocation, prevLocation, 0.01);
        argos::Real dist = Distance(argos::CVector2(currentLocation.GetX(), currentLocation.GetY()), argos::CVector2(prevLocation.GetX(), prevLocation.GetY()));

        argos::LOG << "curr: " << currentLocation.GetX() << std::endl;
        argos::LOG << "prev: " << prevLocation.GetX() << std::endl;

        // out << currentLocation.GetX() << ',' << prevLocation.GetX() << std::endl;
        out << currentLocation.GetX() - prevLocation.GetX() << std::endl;

        // out << abs(currentLocation.GetX() - prevLocation.GetX()) << std::endl;
        // prevMes = dist;
        prevLocation = currentLocation;
    }
    else if(time == 1)
    {
        prevLocation = currentLocation;
    }

    argos::LOG << "dist to goal: " << sqrt(pow(goalPoint.GetX() - currentLocation.GetX(), 2) + pow(goalPoint.GetY() - currentLocation.GetY(), 2)) << std::endl;
    if(sqrt(pow(goalPoint.GetX() - currentLocation.GetX(), 2) + pow(goalPoint.GetY() - currentLocation.GetY(), 2)) <= 0.04999f)
    {
        m_pcWheels->SetLinearVelocity(0,0);
        testComplete = true;
        std::cout << "time: " << time << std::endl;
        out.close();
    }

    // prevLocation = currentLocation;

    //wVel = con.angleControl(robotAngle, bugGoalAngle);

    // std::cout << "test: " << wVel.lWheel << wVel.rWheel << '\n';
    // m_pcWheels->SetLinearVelocity(wVel.lWheel, wVel.rWheel);


    // argos::Real leftWheeleVelocity;
    // argos::Real rightWheeleVelocity;

    // leftWheeleVelocity = wVel.lWheel;
    // rightWheeleVelocity = wVel.rWheel;

    // std::cout << "Diff:  " << abs(bugGoalAngle.GetValue() - robotAngle.GetValue()) << '\n';
    // std::cout << "Thres: " << ANGLE_THRESHOLD << '\n';

    // argos::LOG << "dist to goal: " << sqrt(pow(goalPoint.GetX() - robotPos.GetX(), 2) + pow(goalPoint.GetY() - robotPos.GetY(), 2)) << std::endl;
    // if(sqrt(pow(goalPoint.GetX() - robotPos.GetX(), 2) + pow(goalPoint.GetY() - robotPos.GetY(), 2)) <= 0.04999f)
    // {
    //     m_pcWheels->SetLinearVelocity(0,0);
    //     testComplete = true;
    //     std::cout << "time: " << time << std::endl;
    //     out.close();
    // }
    // else if(abs(bugGoalAngle.GetValue() - robotAngle.GetValue()) >= ANGLE_THRESHOLD)
    // {
    //     leftWheeleVelocity = wVel.lWheel;
    //     rightWheeleVelocity = wVel.rWheel;
    //     //std::cout << leftWheeleVelocity<< ' ' << rightWheeleVelocity <<'\n';
    //     m_pcWheels->SetLinearVelocity(leftWheeleVelocity, -rightWheeleVelocity);
    // }
    // else
    // {
    //     leftWheeleVelocity = wVel.lWheel + 60;
    //     rightWheeleVelocity = wVel.rWheel - 60;
    //     bugAlg.regulateSpeed(m_pcProximity, leftWheeleVelocity, rightWheeleVelocity);
    //     m_pcWheels->SetLinearVelocity(leftWheeleVelocity, rightWheeleVelocity);
    // }

    // std::cout << "vel: " << leftWheeleVelocity <<  " " << rightWheeleVelocity  << std::endl;
}

REGISTER_CONTROLLER(test_controller, "test_controller")
