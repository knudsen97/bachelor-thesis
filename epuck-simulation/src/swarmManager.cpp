#include "swarmManager.h"

void sortClosestBox(std::vector<argos::CBoxEntity*> &boxes, const argos::CVector3& goal)
{
    std::sort(
        std::begin(boxes), 
        std::end(boxes), 
        [goal](argos::CBoxEntity* a, argos::CBoxEntity* b) {
            return 
            argos::Distance(a->GetEmbodiedEntity().GetOriginAnchor().Position, goal) 
            <
            argos::Distance(b->GetEmbodiedEntity().GetOriginAnchor().Position, goal); 
        }
    );
}

void sortClosestObject(std::vector<argos::CPrototypeEntity*> &objects, const argos::CVector3& goal)
{
    std::sort(
        std::begin(objects), 
        std::end(objects), 
        [goal](argos::CPrototypeEntity* a, argos::CPrototypeEntity* b) {
            return 
            argos::Distance(a->GetEmbodiedEntity().GetOriginAnchor().Position, goal) 
            <
            argos::Distance(b->GetEmbodiedEntity().GetOriginAnchor().Position, goal); 
        }
    );
}

swarmManager::swarmManager() :
                            blueGoal (argos::CVector3(0,0,0)),
                            whiteGoal(argos::CVector3(0,0,0)),
                            blueBoxIdx(0),
                            whiteBoxIdx(0),
                            blueObjectIdx(0),
                            whiteObjectIdx(0),
                            firstRun(false)
                            {}

swarmManager::~swarmManager()
{}

void swarmManager::setGoals(argos::CVector3 _blueGoal, argos::CVector3 _whiteGoal)
{
    blueGoal    = _blueGoal;
    whiteGoal   = _whiteGoal;
}


void swarmManager::step()
{
    if(!firstRun)
    {
        if (!swarmBoxes.empty())
        {
            classifyBoxes(swarmBoxes, whiteBoxes, blueBoxes);
            std::thread sortBlue(sortClosestBox, std::ref(blueBoxes), std::ref(blueGoal));
            std::thread sortWhite(sortClosestBox, std::ref(whiteBoxes), std::ref(whiteGoal));
            sortBlue.join();
            sortWhite.join();
            blueServer(3, blueGoal, blueBoxes[blueBoxIdx++], "blue server");
            if (cameraServerLoop::totalClientCount > 3)
            {
                whiteServer(3, whiteGoal, whiteBoxes[whiteBoxIdx++], "white server");
            }
        }
        if (!swarmObjects.empty())        
        {
            classifyBoxes(swarmObjects, whiteObjects, blueObjects);
            std::thread sortBlue(sortClosestObject, std::ref(blueObjects), std::ref(blueGoal));
            std::thread sortWhite(sortClosestObject, std::ref(whiteObjects), std::ref(whiteGoal));
            sortBlue.join();
            sortWhite.join();
            blueServer(3, blueGoal, blueObjects[blueObjectIdx++], "blue server");
            if (cameraServerLoop::totalClientCount > 3)
            {
                whiteServer(3, whiteGoal, whiteObjects[whiteObjectIdx++], "white server");
            }
        }
        

        firstRun = true;
    }

    if (blueServer.jobsDone)
    {
        if (blueBoxIdx < blueBoxes.size())
        {
            blueServer( blueGoal, blueBoxes[blueBoxIdx++]);
        }
        else if (blueObjectIdx < blueObjects.size())
        {
            blueServer( blueGoal, blueObjects[blueObjectIdx++]);
        }
        /* if this swarm shall push them all */
        else if (whiteBoxIdx < whiteBoxes.size())
        {
            blueServer( whiteGoal, whiteBoxes[whiteBoxIdx++]);
        }
        else if (whiteObjectIdx < whiteObjects.size())
        {
            blueServer( whiteGoal, whiteObjects[whiteObjectIdx++]);
        }
    }
    // if (whiteServer.jobsDone)
    // {
    //     if (whiteBoxIdx < whiteBoxes.size())
    //     {
    //         whiteServer( whiteGoal, whiteBoxes[whiteBoxIdx++]);
    //     }
    //     else if (whiteObjectIdx < whiteObjects.size())
    //     {
    //         whiteServer( whiteGoal, whiteObjects[whiteObjectIdx++]);
    //     }
    // }
    blueServer.step();

    if (cameraServerLoop::totalClientCount > 3)
    {
        if (whiteServer.jobsDone)
        {
            if (whiteBoxIdx < whiteBoxes.size())
            {
                whiteServer( whiteGoal, whiteBoxes[whiteBoxIdx++]);
            }
            else if (whiteObjectIdx < whiteObjects.size())
            {
                whiteServer( whiteGoal, whiteObjects[whiteObjectIdx++]);
            }
        }
        whiteServer.step();
    }
    
    // argos::LOG << "white: " << whiteBoxes.size() << std::endl;
    // argos::LOG << "blue: " << blueBoxes.size() << std::endl;
    // argos::LOG << "wGoal: " << whiteGoal << std::endl;
    // argos::LOG << "bGoal: " << blueGoal << std::endl;

}

/**
 * @brief Classifies boxes as either white or blue
 * @param _boxes All the boxes in the simulation
 * @param _whiteObjects Objects with white leds are stored here
 * @param _blueObjects Objects with blue leds are stored here
*/
void swarmManager::classifyBoxes(std::vector<argos::CBoxEntity*> _boxes, 
                                    std::vector<argos::CBoxEntity*> &_whiteObjects, std::vector<argos::CBoxEntity*> &_blueObjects)
{
    argos::CColor ledColor;
    for(argos::CBoxEntity* box : _boxes)
    {
        ledColor = box->GetLEDEquippedEntity().GetLED(0).GetColor(); //Only looking at index 0, because only 1 led per box
        if(ledColor == argos::CColor::WHITE)
        {
            _whiteObjects.push_back(box);
        }
        else if(ledColor == argos::CColor::BLUE)
        {
            _blueObjects.push_back(box);
        }
        else
        {
            argos::LOGERR << "Colour on LED not white/blue\n";
            break;
        }
    }

}

/**
 * @brief Classifies boxes as either white or blue
 * @param _boxes All the boxes in the simulation
 * @param _whiteBoxes Boxes with white leds are stored here
 * @param _blueBoxes Boxes with blue leds are stored here
*/
void swarmManager::classifyBoxes(std::vector<argos::CPrototypeEntity*> _boxes, 
                                    std::vector<argos::CPrototypeEntity*> &_whiteBoxes, std::vector<argos::CPrototypeEntity*> &_blueBoxes)
{
    argos::CColor ledColor;
    for(argos::CPrototypeEntity* box : _boxes)
    {
        ledColor = box->GetLEDEquippedEntity().GetLED(0).GetColor(); //Only looking at index 0, because only 1 led per box
        if(ledColor == argos::CColor::WHITE)
        {
            _whiteBoxes.push_back(box);
        }
        else if(ledColor == argos::CColor::BLUE)
        {
            _blueBoxes.push_back(box);
        }
        else
        {
            argos::LOGERR << "Colour on LED not white/blue\n";
            break;
        }
    }

}
