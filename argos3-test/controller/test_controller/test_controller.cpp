/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

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
        m_pcProximity = GetSensor<CCI_ProximitySensor>("proximity");
        GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    }

 

    void test_controller::ControlStep()
    {
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
    }

   REGISTER_CONTROLLER(test_controller, "test_controller")
