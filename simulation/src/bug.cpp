#include "../inc/bug.h"


// notice that the porximity sensor measures 0 when no object
// 1 when object touch the robot and everything in between
#define NEAR 0.3
#define FAR 0.1


/**
 * This function takes in the positinoing sensor, proximity sensor and a 
 * goal to calculate the orientation, the robot need to be in to drive 
 * in a bug 0 algorithm style movement. The function returns the 
 * orientation in argos::CRadians.
 * 
 * @brief Call bug::move() without instantiating a bug object.
 * 
 * @param proximitySensor Takes the proximity sensor object to the robot.
 * @param positionSensor Takes the positioning sensor object to the robot.
 * @param goal The goal destination of the robot.
 * 
 * @return The orientation of the robot in argos::CRadians
*/
argos::CRadians bug::move(const argos::CCI_FootBotProximitySensor* proximitySensor, const argos::CCI_PositioningSensor* positionSensor, argos::CVector3 goal)
{
    //get readings and initiate variables
    argos::CCI_FootBotProximitySensor prox = *proximitySensor;
    argos::CCI_FootBotProximitySensor::TReadings tProxReads = prox.GetReadings();
    float maxVal = tProxReads[0].Value;
    int indexMax = 0;
    argos::CRadians returnAngle, robotAngle, temp, zAngle;

    //only looking at the front beams
    for(size_t i = 0; i < tProxReads.size(); i++)
        if (tProxReads[i%23].Value > maxVal)
        {
            indexMax = i%23;
            maxVal = tProxReads[i%23].Value;
        }
    
    // hysteresis
    // begin avoiding when NEAR object
    // stop avoiding when FAR object 
    if(tProxReads[indexMax].Value > NEAR) 
        bug::bugMemory_turn = true;
    if(tProxReads[indexMax].Value < FAR)
        bug::bugMemory_turn = false;

    if(bug::bugMemory_turn == true)
    {
        //if angle and such have not been read when entering avoidance state
        if(!bug::bugMemory_objectAngleRead) 
        {
            positionSensor->GetReading().Orientation.ToEulerAngles(robotAngle,temp,temp);
            bug::bugMemory_robotAngle = robotAngle;
            bug::bugMemory_objectAngle = tProxReads[indexMax].Angle;
            bug::bugMemory_objectAngleRead = true;
        }
        //its angle + the angle to object + 90deg
        returnAngle = bug::bugMemory_robotAngle + bug::bugMemory_objectAngle + argos::CRadians(M_PI_2);
    }
    else
    {
        //stolen form @knudsen97 lol
        positionSensor->GetReading().Orientation.ToEulerAngles(temp, temp, zAngle);
        argos::CVector3 posDiff = goal - positionSensor->GetReading().Position;
        returnAngle = argos::ATan2(posDiff.GetY(), posDiff.GetX());
        bug::bugMemory_objectAngleRead = false;
    }
    return returnAngle;
}

//bug memories
argos::CRadians bug::bugMemory_robotAngle = argos::CRadians(0);
argos::CRadians bug::bugMemory_objectAngle = argos::CRadians(0);
bool bug::bugMemory_turn = false;
bool bug::bugMemory_objectAngleRead = false;