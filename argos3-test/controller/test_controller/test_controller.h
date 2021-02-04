#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>


using namespace argos;

class test_controller : public CCI_Controller {

public:

   test_controller();
   ~test_controller(){}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

private:
   CCI_DifferentialSteeringActuator* m_pcWheels;

   Real m_fWheelVelocity;

};


#endif