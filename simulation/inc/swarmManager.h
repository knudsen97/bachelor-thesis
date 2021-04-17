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

    std::vector<argos::CBoxEntity*> swarmBoxes;

private:
    std::vector<argos::CBoxEntity*> whiteBoxes, blueBoxes;
    argos::CVector3 blueGoal, whiteGoal;
};

#endif //SWARMMANGER