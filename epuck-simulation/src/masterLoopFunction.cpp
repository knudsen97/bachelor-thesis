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
            argos::LOG <<"here\n";
            if(objId.compare(2,3,"3d") == 0) // Check that box is not a wall
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
argos::LOG << '\n';
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
      argos::Distance(boxPosition, swarmMan.blueGoal) < GOAL_THRESHOLD ||
      argos::Distance(boxPosition, swarmMan.whiteGoal) < GOAL_THRESHOLD
   )
   {
      //if box not found, add bot to "soon to remove" list
      if(boxFinder == boxesToRemove.end())
      {
         boxesToRemove.push_back({0, boxName, box});
      }
      else //destory if it has lived its live
      {
         argos::LOG << "boxname: " << boxFinder->name << '\n';
         argos::LOG << "TTL: " << boxFinder->time << '\n';
         if (boxFinder->time++ > BOX_TTL)
         {
            boxFinder->box->GetEmbodiedEntity().MoveTo({-1,-1,0}, {0,0,0,0}, false, true);
            boxesToRemove.erase(boxFinder);
            // argos::CLoopFunctions::RemoveEntity(boxFinder->box->GetEmbodiedEntity());
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
      argos::Distance(boxPosition, swarmMan.blueGoal) < GOAL_THRESHOLD ||
      argos::Distance(boxPosition, swarmMan.whiteGoal) < GOAL_THRESHOLD
   )
   {
      //if box not found, add bot to "soon to remove" list
      if(boxFinder == objectsToRemove.end())
      {
         objectsToRemove.push_back({0, boxName, object});
      }
      else //destory if it has lived its live
      {
         argos::LOG << "object name: " << boxFinder->name << '\n';
         argos::LOG << "TTL: " << boxFinder->time << '\n';
         if (boxFinder->time++ > BOX_TTL)
         {
            boxFinder->box->GetEmbodiedEntity().MoveTo({-1,-1,0}, {0,0,0,0}, false, true);
            objectsToRemove.erase(boxFinder);
            // argos::CLoopFunctions::RemoveEntity(boxFinder->box->GetEmbodiedEntity());
         }
         
      }
      
   }

}


/*******************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
