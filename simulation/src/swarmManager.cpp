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

swarmManager::swarmManager() :
                            blueGoal (argos::CVector3(0,0,0)),
                            whiteGoal(argos::CVector3(0,0,0)),
                            blueBoxIdx(0),
                            whiteBoxIdx(0),
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
        classifyBoxes(swarmBoxes, whiteBoxes, blueBoxes);
        std::thread sortBlue(sortClosestBox, std::ref(blueBoxes), std::ref(blueGoal));
        std::thread sortWhite(sortClosestBox, std::ref(whiteBoxes), std::ref(whiteGoal));
        sortBlue.join();
        sortWhite.join();
        blueServer(3, blueGoal, blueBoxes[blueBoxIdx++]);
        whiteServer(3, whiteGoal, whiteBoxes[whiteBoxIdx++]);
        firstRun = true;
    }

    if (blueServer.jobsDone)
    {
        if (blueBoxIdx < blueBoxes.size())
        {
            blueServer(3, blueGoal, blueBoxes[blueBoxIdx++]);
        }
        else if(whiteBoxIdx < whiteBoxes.size())
        {
            blueServer(3, whiteGoal, whiteBoxes[whiteBoxIdx++]);
        }
    }
    if (whiteServer.jobsDone)
    {
        if (whiteBoxIdx < whiteBoxes.size())
        {
            whiteServer(3, whiteGoal, whiteBoxes[whiteBoxIdx++]);
        }
    }
    whiteServer.step();
    blueServer.step();
    
    // std::cout << "white: " << whiteBoxes.size() << std::endl;
    // std::cout << "blue: " << blueBoxes.size() << std::endl;
    // std::cout << "wGoal: " << whiteGoal << std::endl;
    // std::cout << "bGoal: " << blueGoal << std::endl;

}

/**
 * @brief Classifies boxes as either white or blue
 * @param _boxes All the boxes in the simulation
 * @param _whiteBoxes Boxes with white leds are stored here
 * @param _blueBoxes Boxes with blue leds are stored here
*/
void swarmManager::classifyBoxes(std::vector<argos::CBoxEntity*> _boxes, 
                                    std::vector<argos::CBoxEntity*> &_whiteBoxes, std::vector<argos::CBoxEntity*> &_blueBoxes)
{
    argos::CColor ledColor;
    for(argos::CBoxEntity* box : _boxes)
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
