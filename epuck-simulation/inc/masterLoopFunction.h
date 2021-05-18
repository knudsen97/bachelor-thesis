#ifndef __MASTERLOOPFUNCTION_H__
#define __MASTERLOOPFUNCTION_H__

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>

#include "cameraServerLoop.h"
#include "camera.h"
#include "swarmManager.h"
#include "sort_loop_function.h"

#define GOAL_THRESHOLD 0.39999f
#define BOX_TTL 500

using namespace argos;

class masterLoopFunction : public cameraServerLoop, public camera, public swarmManager  
{
private:
    /* data */
    bool firstIteration = true;
    int numBoxes = 0;
    int numObjects = 0;
    void replaceObject(argos::CPrototypeEntity* obj);
    argos::CVector3 generatePosition();
            CPrototypeEntity* obj;

public:
    static size_t argosTime;
    static bool experimentDone;
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
    struct objAtGoal
    {
        int time;
        std::string name;
        argos::CPrototypeEntity* box;
    };
    void removeBoxAtGoal(argos::CBoxEntity* box);
    void removeBoxAtGoal(argos::CPrototypeEntity* object);
    std::vector<boxAtGoal> boxesToRemove;
    std::vector<objAtGoal> objectsToRemove;
    std::vector<argos::CVector3> objectLocations;


};


#endif // __MASTERLOOPFUNCTION_H__