/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

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
                                CVector3(2, 1.7, 0.0), // position
                                CQuaternion(),           // orientation
                                true,                    // movable or not?
                                CVector3(0.1, 0.4, 0.5), // size
                                500.0);                    // mass in kg


        AddEntity(*pcBox);
    }

    test_controller::cPositions test_controller::findCPositions(CBoxEntity* mBox)
    {
        //Find the box' origin/center of mass:
        CVector2 origin = {mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()};
        //Get the Orientation of the box:
        CRadians xAngle, yAngle, zAngle;
        mBox->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
        Real theta = xAngle.GetValue();          //cos and sin does'nt take degrees but radians*180/PI;     //Radians to degrees
        //LOG << "Angle: " << theta << std::endl;

        //Get box dimensions to calculate offset:
        Real xSize = mBox->GetSize().GetX();
        Real ySize = mBox->GetSize().GetY();    

        //TOP RIGHT CORNER:
        Real c1x = origin.GetX() + ((xSize / 2) * cos(theta)) - ((ySize / 2) * sin(theta));
        Real c1y = origin.GetY() + ((xSize / 2) * sin(theta)) + ((ySize / 2) * cos(theta));

        //TOP LEFT CORNER:
        Real c2x  = origin.GetX() - ((xSize / 2) * cos(theta)) - ((ySize / 2) * sin(theta));
        Real c2y  = origin.GetY() - ((xSize / 2) * sin(theta)) + ((ySize / 2) * cos(theta));

        //BOTTOM LEFT CORNER:
        Real c3x = origin.GetX() - ((xSize / 2) * cos(theta)) + ((ySize / 2) * sin(theta));
        Real c3y = origin.GetY() - ((xSize / 2) * sin(theta)) - ((ySize / 2) * cos(theta));

        //BOTTOM RIGHT CORNER:
        Real c4x = origin.GetX() + ((xSize / 2) * cos(theta)) + ((ySize / 2) * sin(theta));
        Real c4y = origin.GetY() + ((xSize / 2) * sin(theta)) - ((ySize / 2) * cos(theta));

        test_controller::cPositions c;
        c.c1 = {c1x, c1y, 0};
        c.c2 = {c2x, c2y, 0};
        c.c3 = {c3x, c3y, 0};
        c.c4 = {c4x, c4y, 0};

        return c;
    }

    //Used to calculate the projection in findPushPoints() function:
    template<class V>
    Real test_controller::projection(V &v1, V &v2)
    {
        //return v1.DotProduct(v2)/(sqrt(pow(v2.GetX(),2)+pow(v2.GetY(),2)));
        return v1.DotProduct(v2)/(abs(v2.GetX()+v2.GetY()));    //Absolute value is taken since we're only interested in the sign of the value.
    }
    std::vector<CVector3> test_controller::findPushPoints(CBoxEntity* mBox, CVector3 goalPoint)
    {
        std::vector<CVector3> validPushPoints;
        test_controller::cPositions c = findCPositions(mBox);

        //Find corner vectors:
        std::array<CVector3,2> c1Vecs, c2Vecs, c3Vecs, c4Vecs;
        c1Vecs = {c.c1 - c.c2, c.c1 - c.c4};
        c2Vecs = {c.c2 - c.c1, c.c2 - c.c3};
        c3Vecs = {c.c3 - c.c2, c.c3 - c.c4};
        c4Vecs = {c.c4 - c.c1, c.c4 - c.c3};

        //For debugging:
        // LOG << "c1Vecs[0]: " << c1Vecs[0] << std::endl;
        // LOG << "c1Vecs[1]: " << c1Vecs[1] << std::endl;

        // LOG << "c2Vecs[0]: " << c2Vecs[0] << std::endl;
        // LOG << "c2Vecs[1]: " << c2Vecs[1] << std::endl;

        // LOG << "c3Vecs[0]: " << c3Vecs[0] << std::endl;
        // LOG << "c3Vecs[1]: " << c3Vecs[1] << std::endl;

        // LOG << "c4Vecs[0]: " << c4Vecs[0] << std::endl;
        // LOG << "c4Vecs[1]: " << c4Vecs[1] << std::endl;

        //Create vector from box to goal:
        CVector3 dvBox = goalPoint - mBox->GetEmbodiedEntity().GetOriginAnchor().Position;
        //LOG << "dvBox: " << dvBox << std::endl;

        //Find pushing points by projecting dvBox vector with each corners vectors. If atleast 1 projection is negative it is a valid point.
        bool debug = false;
        bool c1Found=0, c2Found=0, c3Found=0, c4Found=0;
        for(size_t i = 0; i<2; i++) //TODO: DENNE LØKKE KAN GODT TAGE DET SAMME PUNKT 2 GANGE!!!!!!
        {
            if(projection(dvBox, c1Vecs[i]) < 0 && !c1Found)
            {
                validPushPoints.push_back(c.c1);
                c1Found = 1;
                if(debug)
                    LOG << "Projections corner 1: " << projection(dvBox, c1Vecs[i]) << std::endl;
            }
            if(projection(dvBox, c2Vecs[i]) < 0 && !c2Found)
            {
                validPushPoints.push_back(c.c2);
                c2Found = 1;
                if(debug)
                    LOG << "Projections corner 2: " << projection(dvBox, c2Vecs[i]) << std::endl;
            }
            if(projection(dvBox, c3Vecs[i]) < 0 && !c3Found)
            {
                validPushPoints.push_back(c.c3);
                c3Found = 1;
                if(debug)
                    LOG << "Projections corner 3: " << projection(dvBox, c3Vecs[i]) << std::endl;
            }
            if(projection(dvBox, c4Vecs[i]) < 0 && !c4Found)
            {
                validPushPoints.push_back(c.c4);
                c4Found = 1;
                if(debug)
                    LOG << "Projections corner 4: " << projection(dvBox, c4Vecs[i]) << std::endl;
            }
        }

        return validPushPoints;
    }

    void test_controller::ControlStep()
    {
        const CCI_PositioningSensor::SReading& robotPos = posSensor->GetReading();

        CVector3 goal;
        goal.Set(3.3, 3.3, 0);
        std::vector<CVector3> validPushPoints = test_controller::findPushPoints(pcBox, goal);

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

        CVector3 posDiff = validPushPoints[1] - robotPos.Position;
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
