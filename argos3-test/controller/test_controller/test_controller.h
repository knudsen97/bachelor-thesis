#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <cmath>
#include <string>
#include "planner.h"
//#include "camera.h"

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/generic/simulator/camera_default_sensor.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

using namespace argos;

class test_controller : public CCI_Controller, public CLoopFunctions, public CBoxEntity{

public:


   test_controller();
   ~test_controller(){}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();




private:
   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_PositioningSensor* posSensor;
   CCI_CameraSensor* camSensor;
   
   CBoxEntity* pcBox;

   Real m_fWheelVelocity;

   bool test = false;
   bool test1 = false;

   planner P;
   camera C;

};

#endif