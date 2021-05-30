#ifndef SWARMMANGER_H
#define SWARMMANGER_H

#include "cameraServerLoop.h"

class swarmManager{
public:
    swarmManager();
    ~swarmManager();
    
    void setGoals(argos::CVector3 _blueGoal, argos::CVector3 _whiteGoal);
    void step();

    void classifyBoxes(std::vector<argos::CBoxEntity*> _boxes, std::vector<argos::CBoxEntity*> &_whiteBoxes, std::vector<argos::CBoxEntity*> &_blueBoxes);
    void classifyBoxes(std::vector<argos::CPrototypeEntity*> _boxes, std::vector<argos::CPrototypeEntity*> &_whiteObjects, std::vector<argos::CPrototypeEntity*> &_blueObjects);

    std::vector<argos::CBoxEntity*> swarmBoxes;
    std::vector<argos::CPrototypeEntity*> swarmObjects;

    cameraServerLoop blueServer, whiteServer;
    int blueBoxIdx, whiteBoxIdx;
    int blueObjectIdx, whiteObjectIdx;
    bool firstRun;

    std::vector<argos::CBoxEntity*> whiteBoxes, blueBoxes;
    std::vector<argos::CPrototypeEntity*> whiteObjects, blueObjects;
    argos::CVector3 blueGoal, whiteGoal;
private:

};

#endif //SWARMMANGER