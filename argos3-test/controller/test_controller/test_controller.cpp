/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


using namespace argos;

    test_controller::test_controller() :
        m_pcWheels(NULL),
        m_fWheelVelocity(2.5f),
        posSensor(NULL),
        pcBox(NULL){}

    void test_controller::Init(TConfigurationNode& t_node) 
    {
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        posSensor = GetSensor<CCI_PositioningSensor>("positioning");
        GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

        pcBox = new CBoxEntity("box1",                   // id
                                CVector3(0.5, 0.5, 0.0), // position
                                CQuaternion(),           // orientation
                                true,                    // movable or not?
                                CVector3(0.2, 0.2, 0.5), // size
                                2.0);                    // mass in kg

       AddEntity(*pcBox);
    }

    void test_controller::ControlStep()
    {
        const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();

        LOG << "Box X: " << pcBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() << std::endl;
        LOG << "X: " << robotPos.Position.GetX() << std::endl;
        LOG << "Y: " << robotPos.Position.GetY() << std::endl;

        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }

   REGISTER_CONTROLLER(test_controller, "test_controller")
