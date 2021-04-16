#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <cmath>
#include <string>
#include <thread>
#include "planner.h"
#include "controller.h"
//#include "camera.h"

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/generic/simulator/camera_default_sensor.h>
#include <argos3/core/utility/networking/tcp_socket.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

/* TCP communication */
#include <argos3/core/utility/networking/tcp_socket.h>
#include "../inc/protocol.h"

#define BUFFERSIZE 4
#define V_0 4

//using namespace argos;

class test_controller : public argos::CCI_Controller, public argos::CLoopFunctions, public argos::CBoxEntity { 
public:
   /*Constructor*/
   test_controller();
   ~test_controller(){}

   /*ArgOS functions*/
   virtual void Init(argos::TConfigurationNode& t_node);
   virtual void ControlStep();

   /*Planning functions*/
   bool Planning(argos::CVector3 &goal);
   bool ReadyToPush(const argos::CCI_PositioningSensor::SReading& robotPos, 
                     argos::CVector3& goalPoint, argos::CRadians& desiredAngle, argos::CRadians& robotAngle, int v0 = V_0);

private:
   argos::CCI_DifferentialSteeringActuator* m_pcWheels;
   argos::CCI_PositioningSensor* posSensor;
   argos::CCI_CameraSensor* camSensor;
   argos::CCI_FootBotProximitySensor* proxSensor;
   argos::CBoxEntity* pcBox;
   argos::Real m_fWheelVelocity;
   argos::CVector3 robotPosition;

   /*State machine variables*/
   bool pointReached = false;
   bool planComplete = false;
   bool cornerFound = false;
   bool sendWaitState = false;
   int currentState = 0;

   /*Protocol receive variables*/
   argos::CVector3 goalPointMessage;
   argos::CRadians goalAngle, bugGoalAngle;
   argos::Real pushVelocity;
   bug bugAlg;

   /*Socket variables*/
public:
   void connect();
   static size_t robotBufferSize;
   bool joined = false;
   bool sentPosition = false;


private:
   std::thread connecting;
   argos::CByteArray argosBuffer = argos::CByteArray(BUFFERSIZE);
   argos::CTCPSocket clientSocket;
   protocol connection;
   int m_nStream = 0;

};

#endif