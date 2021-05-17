#ifndef CONTROLLER_H
#define CONTROLLER_H
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <fstream>

#define WHEEL_RADIUS 0.0205f //Found in argos documentation
#define INTERWHEEL_DISTANCE 0.053f 

class controller
{
public:
    
    struct wVelocity
    {
        argos::Real lWheel;
        argos::Real rWheel;
    };

    controller();
    controller(argos::Real _dt, argos::Real _Kp, argos::Real _Ki, argos::Real _Kd);
    ~controller();
    void operator()(argos::Real _dt, argos::Real _Kp, argos::Real _Ki, argos::Real _Kd);

    wVelocity angleControl(argos::CRadians curAngle, argos::CRadians desiredAngle); 
    argos::Real angleControl1(argos::CRadians curAngle, argos::CRadians desiredAngle); 

    wVelocity angleControl(argos::CRadians curAngle, const argos::CVector3 &robPos, argos::CVector3 &goalPos); 

    // std::vector<double> getY(){return y;}
    // std::vector<double> getX(){return x;}
    //std::ofstream outfile;
private:
    /*Controller gains & variables*/
    argos::Real Kp, Ki, Kd;
    argos::Real dt;
    argos::Real integral;

    argos::Real vR, vL; //Right and left wheel velocity
    argos::Real angle; 
    argos::Real preError;

    /*Debugging/plotting vector*/
    // static std::vector<double> y,x;
};

#endif