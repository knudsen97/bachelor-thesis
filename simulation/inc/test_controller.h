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

using namespace argos;

class test_controller : public CCI_Controller, public CLoopFunctions, public CBoxEntity { 

public:


   test_controller();
   ~test_controller(){}


   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();
   bool Planning(argos::CVector3 &goal);
   bool ReadyToPush(const CCI_PositioningSensor::SReading& robotPos, 
                     argos::CVector3& goalPoint, argos::CRadians& desiredAngle, argos::CRadians& robotAngle, int v0 = V_0);




private:
   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_PositioningSensor* posSensor;
   CCI_CameraSensor* camSensor;
   CCI_FootBotProximitySensor* proxSensor;
   
   CBoxEntity* pcBox;

   Real m_fWheelVelocity;
   CVector3 robotPosition;

   bool test = false;
   bool test1 = false;

   // planner P;
   // camera C;
   //cv::Mat map;
   bool pointReached = false;
   bool planComplete = false;
   bool cornerFound = false;

   //std::vector<cv::Point> subGoals;
   size_t i = 0;

   int currentState = 0;
   bool positionSendt = false;


   argos::CVector3 goalPointMessage;
   argos::CRadians goalAngle, bugGoalAngle;
   bug bugAlg;

   //socket
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