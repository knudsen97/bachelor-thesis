/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

    test_controller::test_controller() :
        m_pcWheels(NULL),
        m_fWheelVelocity(2.5f){}
        // m_pcProximity(NULL),
        // m_cAlpha(10.0f),
        // m_fDelta(0.5f),
        // m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
        //                         ToRadians(m_cAlpha)) {}

    void test_controller::Init(TConfigurationNode& t_node) 
    {
        m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    }

    void test_controller::ControlStep()
    {
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }

   REGISTER_CONTROLLER(test_controller, "test_controller")
