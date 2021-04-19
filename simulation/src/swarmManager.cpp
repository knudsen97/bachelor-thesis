#include "swarmManager.h"

bool test = false;
bool test2 = false;

swarmManager::swarmManager() :
                            blueGoal (argos::CVector3(0,0,0)),
                            whiteGoal(argos::CVector3(0,0,0))
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
    if(!test)
    {
        classifyBoxes(swarmBoxes, whiteBoxes, blueBoxes);
        test = true;
        blueServer(3, blueGoal, blueBoxes[0]);
    }
    if (blueServer.jobsDone && !test2)
    {
        argos::LOG << "-----------------------------------------------------------------------------\n";
        test2 = true;
        blueServer(3, whiteGoal, whiteBoxes[0]);
    }
    argos::LOG << "server done: " << blueServer.jobsDone << '\n';
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
