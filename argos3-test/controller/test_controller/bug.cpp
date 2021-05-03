#include "bug.h"

 /* for reference of the beam index and direction
 *
 *              front
 *
 *               0 23
 *             1     22
 *           2         21
 *         3             20      r
 * l     4                 19    i
 * e   5                     18  g
 * f   6                     17  h
 * t     7                 16    t
 *         8             15
 *           9         14
 *            10     13
 *              11 12
 *
 *              back
 */


// notice that the porximity sensor measures 0 when no object
// 1 when object touch the robot and everything in between
#define NEAR 0.2
#define FAR 0
#define COEFF 0.1

//states
#define DRIVING 0
#define AVOIDING 1

//the graph for this be seen here
//https://www.google.com/search?q=(0.1)%2F(x%2B0.1)
argos::Real f(argos::Real x)
{
    return (COEFF)/(x+COEFF);
}



bug::bug() 
{
    bug::bugMemory_robotAngle = argos::CRadians(0);
    bug::bugMemory_objectAngle = argos::CRadians(0);
    bug::bugMemory_turn = false;
    bug::bugMemory_objectAngleRead = false;
    bug::bugMemory_state = DRIVING;
    bug::bugMemory_maxVal = 0;
    bug::returnAngle;
}

bug::~bug()
{

}


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
argos::CRadians bug::move(const argos::CCI_EPuckProximitySensor* proximitySensor, const argos::CCI_PositioningSensor* positionSensor, argos::CVector3 goal)
{
    //get readings and initiate variables
    argos::CCI_EPuckProximitySensor prox = *proximitySensor;
    argos::CCI_EPuckProximitySensor::TReadings tProxReads = prox.GetReadings();
    bool free = true;
    int indexMax = 0, indexObjectiveAngle = 0;
    argos::CRadians robotAngle, temp, temp2, objectiveAngle, closestToObjectiveAngleAngle;

    // calculate the angle to the objective and find robot angle (stolen form @knudsen97 lol)
    positionSensor->GetReading().Orientation.ToEulerAngles(robotAngle, temp, temp2);
    argos::CVector3 posDiff = goal - positionSensor->GetReading().Position;
    objectiveAngle = argos::ATan2(posDiff.GetY(), posDiff.GetX());

    //only looking at the front beams 19-20-21-22-23-0-1-2-3-4
    int beamcount = 6; //number of beams to read
    int beamOffset = 5; //offset where to start reading (increments from here)
    int totalBeamCount = 8; //the total number of beams



    switch (bugMemory_state)
    {   
        case DRIVING:
        {
            bugMemory_maxVal = 0;
            
            for(size_t i = 0; i < beamcount; i++)
            {
                int index = ((i+beamOffset)%totalBeamCount);
                if (tProxReads[index].Value > bugMemory_maxVal)
                {
                    indexMax = index;
                    bugMemory_maxVal = tProxReads[index].Value;
                }
            }
            if (bugMemory_maxVal > NEAR)
            {
                bugMemory_state = AVOIDING;
            }
            else
            {
                returnAngle = objectiveAngle;
            }
            break;
        }
        case AVOIDING:
        {
            argos::LOG << "avoid\n";
            closestToObjectiveAngleAngle.SetValue(M_PI*2);
    
            for(size_t i = 0; i < tProxReads.size(); i++)
            {
                //calculate beamangle relative to global coordinate.
                int index = (i%totalBeamCount);
                argos::CRadians beamAngle = tProxReads[index].Angle+robotAngle;

                //wrap beamangle arround to +- pi
                while (beamAngle > argos::CRadians(M_PI))
                    beamAngle -= argos::CRadians(M_PI*2);

                while (beamAngle < argos::CRadians(-M_PI))
                    beamAngle += argos::CRadians(M_PI*2);
                
                //calculate angle difference (taking into account the +- PI angle)
                argos::CRadians angleDifference = argos::CRadians{objectiveAngle.GetValue() - beamAngle.GetValue()};
                angleDifference += 
                    (angleDifference > argos::CRadians(M_PI)) ? (argos::CRadians(-M_PI*2)) : 
                    (angleDifference < argos::CRadians{-M_PI}) ? (argos::CRadians(M_PI*2)) : argos::CRadians(0);      

                //calculate the index for which beam leads to objective
                if (abs(angleDifference.GetValue()) < closestToObjectiveAngleAngle.GetValue())
                {
                    indexObjectiveAngle = index;
                    closestToObjectiveAngleAngle.SetValue(abs(angleDifference.GetValue()));
                }

                //check front beam for more obstacles
                if (i < beamcount) 
                {
                    index = ((i+beamOffset)%totalBeamCount);
                    if (tProxReads[index].Value > bugMemory_maxVal)
                    {
                        indexMax = index;
                        bugMemory_maxVal = tProxReads[index].Value;
                        if (index > 11)
                            returnAngle = tProxReads[indexMax].Angle + robotAngle + argos::CRadians(M_PI_2); //turn left
                        else
                            returnAngle = tProxReads[indexMax].Angle + robotAngle + argos::CRadians(-M_PI_2); //turn right
                    }
                }
            }

            //check if obstacle is still in the way
            int cone = 4;
            int index = 0;
            int tempIndex = 0;
            index = indexObjectiveAngle - (cone/2);
            index += (index < 0) ? (totalBeamCount) : (0);
            for (size_t i = 0; i <= cone; i++)
            {
                tempIndex = (index + i)%totalBeamCount;
                
                free &= (tProxReads[tempIndex].Value == 0);
                
            }
            
            //start driving if there is no obstacle in the way to objective
            if (free)
            {
                returnAngle = objectiveAngle;
                bugMemory_state = DRIVING;
            }

            break;
        }
        default:
        {
            returnAngle = objectiveAngle;
            argos::LOGERR << "error in bug: bug Memory_state was " << bugMemory_state << '\n';;
            break;
        }
    }

    while (returnAngle > argos::CRadians(M_PI))
        returnAngle -= argos::CRadians(M_PI*2);

    while (returnAngle < argos::CRadians(-M_PI))
        returnAngle += argos::CRadians(M_PI*2);
    
    return returnAngle;
}

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
 * @param f_left_velocity The output for left wheel.
 * @param f_right_velocity The output for right wheel.
*/
void bug::regulateSpeed(const argos::CCI_EPuckProximitySensor* proximitySensor, argos::Real& f_left_velocity, argos::Real& f_right_velocity )
{
    //get readings and initiate variables
    argos::CCI_EPuckProximitySensor prox = *proximitySensor;
    argos::CCI_EPuckProximitySensor::TReadings tProxReads = prox.GetReadings();
    float maxVal = tProxReads[0].Value;
    int indexMax = 0;
    argos::CRadians returnAngle, robotAngle, temp, zAngle;
    //only regulate speed if the robot is not turning
    if (abs(f_left_velocity) - abs(f_right_velocity) > 0.01)
    {
        //only looking at the front beams 2-1-0-23-22-21
        for(size_t i = 0; i < 6; i++)
            if (tProxReads[(i+21)%24].Value > maxVal)
            {
                indexMax = (i+21)%24;
                maxVal = tProxReads[(i+21)%24].Value;
            }
        f_left_velocity = f_left_velocity * f(maxVal);
        f_right_velocity = f_right_velocity * f(maxVal);
    }
    
    
    
}

// //bug memories
// argos::CRadians bug::bugMemory_robotAngle = argos::CRadians(0);
// argos::CRadians bug::bugMemory_objectAngle = argos::CRadians(0);
// bool bug::bugMemory_turn = false;
// bool bug::bugMemory_objectAngleRead = false;
// int bug::bugMemory_state = DRIVING;
// float bug::bugMemory_maxVal = 0;
// argos::CRadians bug::returnAngle;