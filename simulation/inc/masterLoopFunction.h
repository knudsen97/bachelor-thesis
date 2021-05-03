#ifndef __MASTERLOOPFUNCTION_H__
#define __MASTERLOOPFUNCTION_H__

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>

#include "cameraServerLoop.h"
#include "camera.h"
#include "swarmManager.h"
#include "sort_loop_function.h"

#define GOAL_THRESHOLD 0.19999f
#define BOX_TTL 200

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
    virtual void PostStep();

    void placeBox();
    masterLoopFunction(/* args */);
    ~masterLoopFunction();

/*remove box at goal stuff*/
    struct boxAtGoal
    {
        int time;
        std::string name;
        argos::CBoxEntity* box;
    };
    void removeBoxAtGoal(argos::CBoxEntity* box);
    std::vector<boxAtGoal> boxesToRemove;



};


#endif // __MASTERLOOPFUNCTION_H__