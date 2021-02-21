/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#define PI 3.14159265

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
                                CVector3(2, 1.7, 0.0), // position
                                CQuaternion(),           // orientation
                                true,                    // movable or not?
                                CVector3(0.1, 0.4, 0.5), // size
                                500.0);                    // mass in kg


        AddEntity(*pcBox);

        cv::Mat map;
        map = C.camera::GetPlot();
        //cv::imshow("test", map);

        CVector3 goal, start;
        goal.Set(15,15,0);
        start.Set(5,5,0);
        map = P.planner::Wavefront(map,start,goal);
        P.planner::Pathfinder(map, start, goal);
    }

 

    void test_controller::ControlStep()
    {
        const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();

        CVector3 goal;
        goal.Set(3.3, 3.3, 0);
        //std::vector<CVector3> validPushPoints = this->A.findPushPoints(pcBox, goal);
        std::vector<CVector3> validPushPoints;
        validPushPoints = P.FindPushPoints(pcBox,goal);
        //std::vector<CVector3> validPushPoints = findPushPoints(pcBox, goal);

        //To draw the points in argos. For debugging
        // if(!test && validPushPoints.size() != 0)
        // {      
        //     for(size_t i=0; i<validPushPoints.size(); i++)
        //     {
        //         LOG << "Corner" + std::to_string(i+1) + " " << validPushPoints[i] << std::endl; 
        //         CBoxEntity* pushCorners;
        //         pushCorners = new CBoxEntity("corner"+std::to_string(i+1),  // id
        //                             validPushPoints[i],                     // position
        //                             CQuaternion(),                          // orientation
        //                             false,                                  // movable or not?
        //                             CVector3(0.01, 0.01, 0.8),              // size
        //                             2000.0);                                // mass in kg
        //         AddEntity(*pushCorners);
        //     }
        //     test = true;
        // }

        //Implementation of simple bug 0 algorithm:
        CRadians xAngle, yAngle, zAngle;
        robotPos.Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
        // LOG << "xAngle: " << xAngle.GetValue()*180/PI << std::endl;
        // LOG << "yAngle: " << yAngle.GetValue()*180/PI << std::endl;
        // LOG << "zAngle: " << zAngle.GetValue()*180/PI << std::endl;

        CVector3 posDiff = validPushPoints[0] - robotPos.Position;
        Real angleDiff = (xAngle - ATan2(posDiff.GetY(), posDiff.GetX())).GetValue();
        LOG << angleDiff << std::endl;
        Real angleTH = 0.0399f;
        if(angleDiff < -angleTH && !test1)
        {
            LOG << "Go CCW\n";
            m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
        }
        else if(angleDiff > angleTH && !test1)
        {
            LOG << "GO CW\n";
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
        }
        // else if(posDiff.GetX() < 0.09999f)
        // {
        //     LOG << "STOP\n";
        //     m_pcWheels->SetLinearVelocity(0, 0);
        //     test1 = true;
        // }
        else if(!test1)
        {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            LOG << "Go straight\n";
        }
        // LOG << "posDiff: " << posDiff << std::endl;
        // LOG << "angleDiff: " << angleDiff << std::endl;


    }

   REGISTER_CONTROLLER(test_controller, "test_controller")
