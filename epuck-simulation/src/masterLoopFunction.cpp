#include "../inc/masterLoopFunction.h"
int footbotCount = 0;
bool boxExist = true;

masterLoopFunction::masterLoopFunction() 
{
    
}

masterLoopFunction::~masterLoopFunction() 
{
    
}

void masterLoopFunction::Init(argos::TConfigurationNode& t_tree) {

   
   
   //get portnumber
   try {
         argos::TConfigurationNodeIterator itDistr;         
         /* Get current node */
         for(itDistr = itDistr.begin(&t_tree);
            itDistr != itDistr.end();
            ++itDistr) 
            {
            argos::TConfigurationNode& tDistr = *itDistr;
            if(itDistr->Value() == "server") {
                
               argos::GetNodeAttribute(tDistr, "port", cameraServerLoop::portnumber);
            }
         }
      }  
   catch(argos::CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }


   argos::CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("e-puck");
   for (argos::CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
      footbotCount++;

   // server(footbotCount);
   // server.connect();

   /* Set goals for white and blue boxes */
   argos::CVector3 blueGoal (1,5,0);
   argos::CVector3 whiteGoal(5,5,0);
   swarmMan.setGoals(blueGoal, whiteGoal);

   // CSortLoopFunction S;
   // S.PlaceBox(argos::CVector3(0.5,0.5,0), 
   //            argos::CVector3(4,5.5,0), 
   //            argos::CVector3(0.3, 0.3, 0.2), 
   //            argos::CVector3(0.5, 0.5, 0.2), 
   //            whiteGoal, blueGoal, 6, 0.9999f);
   cameraServerLoop::establishConnection(footbotCount);
}

void masterLoopFunction::PreStep() 
{
  /* Update camera objects each iteration*/
   for (size_t i = 0; i < camera::objectContainer.size(); i++)
      camera::objectContainer[i]->step();
      
   
   /* First iteration scan the map for boxes and update swarmManger */
   if(firstIteration)
   {
      //relocate 3d objects
      objectLocations.clear();
      try
      {
         CSpace::TMapPerType& objMap = GetSpace().GetEntitiesByType("prototype");
         for (CSpace::TMapPerType::iterator i = objMap.begin(); i != objMap.end(); ++i)
         {
            CPrototypeEntity* obj = any_cast<CPrototypeEntity*>(i->second);
            
            replaceObject(obj);
         }
      }
      catch(const std::exception& e)
      {
         std::cerr << e.what() << '\n';
      }

      try
      {
         argos::CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
         for(argos::CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator)
         {
            argos::CBoxEntity* box = argos::any_cast<argos::CBoxEntity*>(iterator->second);

            std::string boxId = box->GetId();

            if(boxId.compare(0,3,"box") == 0) // Check that box is not a wall
               swarmMan.swarmBoxes.push_back(box);
         }
         numBoxes = swarmMan.swarmBoxes.size();
      }
      catch(const std::exception& e)
      {
         std::cerr << e.what() << '\n';
      }
      
      try
      {
         CSpace::TMapPerType& objMap = GetSpace().GetEntitiesByType("prototype");
         for (CSpace::TMapPerType::iterator i = objMap.begin(); i != objMap.end(); ++i)
         {
            CPrototypeEntity* obj = any_cast<CPrototypeEntity*>(i->second);
            std::string objId = obj->GetId();
            swarmMan.swarmObjects.push_back(obj);
         }

         numObjects = swarmMan.swarmObjects.size();
      }
      catch(const std::exception& e)
      {
         std::cerr << e.what() << '\n';
      }

      firstIteration = false;
   }

   for(auto box : swarmMan.swarmBoxes)
      removeBoxAtGoal(box);
   for(auto obj : swarmMan.swarmObjects)
      removeBoxAtGoal(obj);
   
   
   swarmMan.step();

   
}

void masterLoopFunction::PostStep() 
{
   if (numBoxes != 0 || numObjects != 0)
      argosTime++;
   else
   {
      argos::LOG << "time took: " << argosTime << '\n';   
      experimentDone = true;
   }

   // argos::LOG << "time: " << argosTime << '\n';   
   // argos::LOG << "box: " << numBoxes << '\n';   
   // argos::LOG << "obj: " << numObjects << '\n';   

// argos::LOG << '\n';
}



void masterLoopFunction::removeBoxAtGoal(argos::CBoxEntity* box)
{

   argos::CVector3 boxPosition = box->GetEmbodiedEntity().GetOriginAnchor().Position;
   std::string boxName = box->GetId();

   std::vector<boxAtGoal>::iterator boxFinder = std::find_if(
      boxesToRemove.begin(), 
      boxesToRemove.end(), 
      [boxName](const boxAtGoal& boxesToRemove)
		{return boxesToRemove.name == boxName; }
   );

   if (
      argos::Distance(boxPosition, swarmMan.blueGoal) < GOAL_THRESHOLD+0.1 ||
      argos::Distance(boxPosition, swarmMan.whiteGoal) < GOAL_THRESHOLD+0.1
   )
   {
      //if box not found, add bot to "soon to remove" list
      if(boxFinder == boxesToRemove.end())
      {
         boxesToRemove.push_back({0, boxName, box});
      }
      else //destory if it has lived its live
      {
         // argos::LOG << "boxname: " << boxFinder->name << '\n';
         // argos::LOG << "TTL: " << boxFinder->time << '\n';
         if (boxFinder->time++ > BOX_TTL)
         {
            boxFinder->box->GetEmbodiedEntity().MoveTo({-1,-1,0}, {0,0,0,0}, false, true);
            boxesToRemove.erase(boxFinder);
            // argos::CLoopFunctions::RemoveEntity(boxFinder->box->GetEmbodiedEntity());
            numBoxes--;
         }
         
      }
      
   }

}

void masterLoopFunction::removeBoxAtGoal(argos::CPrototypeEntity* object)
{

   argos::CVector3 boxPosition = object->GetEmbodiedEntity().GetOriginAnchor().Position;
   std::string boxName = object->GetId();

   std::vector<objAtGoal>::iterator boxFinder = std::find_if(
      objectsToRemove.begin(), 
      objectsToRemove.end(), 
      [boxName](const objAtGoal& objectsToRemove)
		{return objectsToRemove.name == boxName; }
   );

   if (
      argos::Distance(boxPosition, swarmMan.blueGoal) < GOAL_THRESHOLD+0.1 ||
      argos::Distance(boxPosition, swarmMan.whiteGoal) < GOAL_THRESHOLD+0.1
   )
   {
      //if box not found, add bot to "soon to remove" list
      if(boxFinder == objectsToRemove.end())
      {
         objectsToRemove.push_back({0, boxName, object});
      }
      else //destory if it has lived its live
      {
         // argos::LOG << "object name: " << boxFinder->name << '\n';
         // argos::LOG << "TTL: " << boxFinder->time << '\n';
         if (boxFinder->time++ > BOX_TTL)
         {
            boxFinder->box->GetLEDEquippedEntity().GetLED(0).Destroy();
            boxFinder->box->GetEmbodiedEntity().MoveTo({-1,-1,0}, {0,0,0,0}, false, true);
            objectsToRemove.erase(boxFinder);
            // argos::CLoopFunctions::RemoveEntity(boxFinder->box->GetEmbodiedEntity());
            numObjects--;
         }
         
      }
      
   }

}

//its re-place not replace
void masterLoopFunction::replaceObject(argos::CPrototypeEntity* object) 
{
   CRadians currentOrientation, non;
   object->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(currentOrientation, non, non);

   //generate random position
   argos::CVector3 newPosition = generatePosition();

   //get object goal
   argos::CColor ledColor;
   argos::CVector3 goal;
   ledColor  = object->GetLEDEquippedEntity().GetLED(0).GetColor();
   if(ledColor == argos::CColor::WHITE)
      goal = swarmMan.whiteGoal;
   else if(ledColor == argos::CColor::BLUE)
      goal = swarmMan.blueGoal;
   else
         argos::LOGERR << "Colour on LED not white/blue\n";
   
   //calculate optimal angle
   argos::CQuaternion orientation;
   argos::CRadians optimalAngle = currentOrientation + argos::ATan2(goal.GetY() - newPosition.GetY(), goal.GetX() - newPosition.GetX());
   // argos::LOG << object->GetId() << " position: " << newPosition << '\n';
   // argos::LOG << object->GetId() << " goal: " << goal << '\n';
   // argos::LOG << object->GetId() << " angle: " << argos::ATan2(goal.GetY() - newPosition.GetY(), goal.GetX() - newPosition.GetX()) << '\n';

   orientation.FromEulerAngles(optimalAngle, non, non);
   object->GetEmbodiedEntity().MoveTo(newPosition,orientation, false, true);
   // argos::LOG <<"movement: " << object->GetEmbodiedEntity().MoveTo(newPosition,orientation, false, true) << '\n';
}

argos::CVector3 masterLoopFunction::generatePosition() 
{
   
   argos::CRange<argos::Real> x_range(0.5, 5), y_range(2.5, 4);
   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");         
   argos::CVector3 newPosition(0,0,0);
   bool run = true;
   argos::Real dist;
   bool distant = true;
   int count = 0;
   // argos::LOG << "loops: " << count << '\n';
   
   while(run)
   {
      newPosition.SetX(pcRNG->Uniform(x_range));
      newPosition.SetY(pcRNG->Uniform(y_range));
      // argos::LOG << newPosition << '\n';
      for(argos::CVector3 objectLocation : objectLocations)
      {
         dist = argos::Distance(objectLocation, newPosition);
         distant &= (dist > 1); //set minimum distance here
      }
      if(distant)
         run = false;
      // else
      //    pcRNG = CRandom::CreateRNG("argos"); //Update random seed
      
      if (count > 10000) //if max runs exceeds
      {
         run = false;
         argos::LOGERR << "max iteration exceeded in placing objec at random";
         throw std::runtime_error("max iteration exceeded in placing objec at random");
      }
      count++;
      distant = true;
   }
   
   // argos::LOG << "loops: " << count << '\n';
   objectLocations.push_back(newPosition);
   return newPosition;
}

size_t masterLoopFunction::argosTime = 0;
bool masterLoopFunction::experimentDone = false;
/*******************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
