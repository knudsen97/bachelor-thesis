#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "bug.h"
#include "camera.h"


#define WHEEL_RADIUS 0.029112741f
#define INTERWHEEL_DISTANCE 0.14f 

class controller
{
    public:
    //controller();
    controller(argos::Real _dt, argos::Real _Kp, argos::Real _Ki, argos::Real _Kd);

    ~controller();

    std::array<argos::Real, 2> angleControl(argos::CRadians curAngle, const argos::CVector3 &robPos, argos::CVector3 &goalPos); //argos::CRadians curAngle);

    private:
    argos::Real Kp, Ki, Kd;
    argos::Real dt;
    argos::Real integral;

    argos::Real vR, vL;
    argos::Real angle;
    argos::Real preError;

};

#endif