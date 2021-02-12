/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <cmath>

#define PI 3.14159265
using namespace argos;

    test_controller::test_controller() :
        m_pcWheels(NULL),
        m_fWheelVelocity(2.5f),
        posSensor(NULL),
        pcBox(NULL){}
        //camSensor(NULL){}

    void test_controller::Init(TConfigurationNode& t_node) 
    {
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        posSensor = GetSensor<CCI_PositioningSensor>("positioning");
        //camSensor = GetSensor<CCI_CameraSensor>("camera0");

        GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

        pcBox = new CBoxEntity("box1",                   // id
                                CVector3(0.5, 0.2, 0.0), // position
                                CQuaternion(),           // orientation
                                true,                    // movable or not?
                                CVector3(0.1, 0.4, 0.5), // size
                                2000.0);                    // mass in kg


       AddEntity(*pcBox);
    }

    test_controller::cPositions test_controller::findCPositions(CBoxEntity* mBox)
    {
        //Find the box' origin/center of mass:
        CVector2 origin = {mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()};
        
        //Get the Orientation of the box:
        CRadians xAngle, yAngle, zAngle;
        mBox->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
        Real theta = xAngle.GetValue()*180/PI;     //Radians to degrees
        LOG << "Angle: " << theta << std::endl;

        //Get box dimensions to calculate offset:
        Real xSize = mBox->GetSize().GetX();
        Real ySize = mBox->GetSize().GetY();    

        //TOP RIGHT CORNER:
        Real c1x = origin.GetX() + ((ySize / 2) * cos(theta)) - ((xSize / 2) * sin(theta));
        Real c1y = origin.GetY() + ((ySize / 2) * sin(theta)) + ((xSize / 2) * cos(theta));

        //TOP LEFT CORNER:
        Real c2x  = origin.GetX() - ((ySize / 2) * cos(theta)) - ((xSize / 2) * sin(theta));
        Real c2y  = origin.GetY() - ((ySize / 2) * sin(theta)) + ((xSize / 2) * cos(theta));

        //BOTTOM LEFT CORNER:
        Real c3x = origin.GetX() - ((ySize / 2) * cos(theta)) + ((xSize / 2) * sin(theta));
        Real c3y = origin.GetY() - ((ySize / 2) * sin(theta)) - ((xSize / 2) * cos(theta));

        //BOTTOM RIGHT CORNER:
        Real c4x = origin.GetX() + ((ySize / 2) * cos(theta)) + ((xSize / 2) * sin(theta));
        Real c4y = origin.GetY() + ((ySize / 2) * sin(theta)) - ((xSize / 2) * cos(theta));

        test_controller::cPositions c;
        c.c1 = {c1x, c1y};
        c.c2 = {c2x, c2y};
        c.c3 = {c3x, c3y};
        c.c4 = {c4x, c4y};

        return c;
    }

    void test_controller::ControlStep()
    {
        const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();

        test_controller::cPositions c = findCPositions(pcBox);

        LOG << "Robot: " << robotPos.Position << std::endl;
        LOG << "c1: " << c.c1 << std::endl;
        LOG << "c2: " << c.c2 << std::endl;
        LOG << "c3: " << c.c3 << std::endl;
        LOG << "c4: " << c.c4 << std::endl;


        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }

   REGISTER_CONTROLLER(test_controller, "test_controller")
