#ifndef __MASTERLOOPFUNCTION_H__
#define __MASTERLOOPFUNCTION_H__

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>

#include "cameraServerLoop.h"
#include "camera.h"
#include "swarmManager.h"

using namespace argos;

class masterLoopFunction : public cameraServerLoop, public camera, public swarmManager  
{
private:
    /* data */
    bool firstIteration = true;
    int numBoxes = 0;
public:
    cameraServerLoop server;
    swarmManager swarmMan;
    virtual void Init(argos::TConfigurationNode& t_tree);
    virtual void PreStep();

    masterLoopFunction(/* args */);
    ~masterLoopFunction();
};


#endif // __MASTERLOOPFUNCTION_H__