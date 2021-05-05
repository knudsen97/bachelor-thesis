// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_proximity_default_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>

class bug
{
public:
    bug();
    ~bug();
    /**
     * This function takes in the positinoing sensor, proximity sensor and a 
     * goal to calculate the orientation, the robot need to be in to drive 
     * in a bug 0 algorithm style movement. The function returns the 
     * orientation in argos::CRadians.
     * 
     * @brief Call move() with an instance of an object.
     * 
     * @param proximitySensor Takes the proximity sensor object to the robot.
     * @param positionSensor Takes the positioning sensor object to the robot.
     * @param goal The goal destination of the robot.
     * 
     * @return The orientation of the robot in argos::CRadians
    */
    // argos::CRadians move(const argos::CCI_FootBotProximitySensor* proximitySensor, const argos::CCI_PositioningSensor* positionSensor, argos::CVector3 goal);
    argos::CRadians move(const argos::CCI_EPuckProximitySensor* proximitySensor, const argos::CCI_PositioningSensor* positionSensor, argos::CVector3 goal);

    void regulateSpeed(const argos::CCI_EPuckProximitySensor* proximitySensor, argos::Real& f_left_velocity, argos::Real& f_right_velocity );
    
    //bug memories
    argos::CRadians bugMemory_robotAngle;
    argos::CRadians bugMemory_objectAngle;
    bool bugMemory_objectAngleRead;
    bool bugMemory_turn;
    int bugMemory_state;
    float bugMemory_maxVal;
    argos::CRadians returnAngle;


};


