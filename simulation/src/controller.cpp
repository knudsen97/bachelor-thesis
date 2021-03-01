#include "controller.h"

/**
 * Controller constructor
 * @param _dt is the sampling rate
 * @param _Kp is the proportional gain
 * @param _Ki is the integral gain
 * @param _Kd is the differentiate gain
*/
controller::controller(argos::Real _dt, argos::Real _Kp, argos::Real _Ki, argos::Real _Kd) :
    dt(_dt),
    Kp(_Kp),
    Kd(_Kd),
    Ki(_Ki),
    preError(0),
    integral(0)
{}

controller::~controller(){}

/**
 * Angle control
 * @param curAngle is the robots current orientation
 * @param robPos is the location of the robot
 * @param goalPos is the goal location
*/
std::array<argos::Real, 2> controller::angleControl(argos::CRadians curAngle, const argos::CVector3 &robPos, argos::CVector3 &goalPos)
{
    //TODO: Indtil videre kører jeg bare med konstant hastighed maybe fix that?
    //Calculate the angle between robot position and goal position:
    argos::CRadians desiredAngle;
    desiredAngle = argos::ATan2(goalPos.GetY()-robPos.GetY(), goalPos.GetX() - robPos.GetX());
    
    //To account for -pi to pi transition:
    argos::Real sign = desiredAngle >= curAngle ? 1.0f : -1.0f;
        //Calculate the error:
    argos::Real error = (desiredAngle - curAngle).GetValue();
    argos::Real S = -sign * M_PI * 2;
    error = (abs(S+error) < abs(error)) ? S+error : error;
    
    //Proportional part
    argos::Real Po = Kp*error;
    //Integral part
    integral += error*dt;
    argos::Real Io = 0; //Ki*integral;
    //Differentiate part
    argos::Real Do = Kd * (error - preError)/dt;

    //Define controller
    argos::Real K = Po + Io + Do;

    //The controller is set to omega and used to calculate the speed of 
    //the individual wheels to obtain the desired angle
    argos::Real omega = K;
    argos::Real v = WHEEL_RADIUS/2 * (this->vR+this->vL);
    this->vR = (2*v+omega*INTERWHEEL_DISTANCE) / 2*WHEEL_RADIUS;
    this->vL = (2*v-omega*INTERWHEEL_DISTANCE) / 2*WHEEL_RADIUS;

    preError = error;
    return {this->vL, this->vR};
}
