#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "bug.h"
#include "camera.h"
#include "../inc/matplotlibcpp.h"
namespace plt = matplotlibcpp;

#define WHEEL_RADIUS 0.029112741f
#define INTERWHEEL_DISTANCE 0.14f 

class controller
{
    public:

    struct wVelocity
    {
        argos::Real lWheel;
        argos::Real rWheel;
    };

    //controller();
    controller(argos::Real _dt, argos::Real _Kp, argos::Real _Ki, argos::Real _Kd);

    ~controller();

    wVelocity angleControl(argos::CRadians curAngle, argos::CRadians desiredAngle); 
    wVelocity angleControl(argos::CRadians curAngle, const argos::CVector3 &robPos, argos::CVector3 &goalPos); 

    std::vector<double> getY(){return y;}
    std::vector<double> getX(){return x;}

    private:
    argos::Real Kp, Ki, Kd;
    argos::Real dt;
    argos::Real integral;

    argos::Real vR, vL;
    argos::Real angle;
    argos::Real preError;

    static std::vector<double> y,x;
};

#endif