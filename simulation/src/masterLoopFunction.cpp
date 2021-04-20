#include "../inc/masterLoopFunction.h"
int footbotCount = 0;

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


   argos::CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("foot-bot");
   for (argos::CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
      footbotCount++;

   // server(footbotCount);
   // server.connect();

   /* Set goals for white and blue boxes */
   argos::CVector3 blueGoal (1,5,0);
   argos::CVector3 whiteGoal(5,5,0);
   swarmMan.setGoals(blueGoal, whiteGoal);

   CSortLoopFunction S;
   S.PlaceBox(argos::CVector3(0.5,0.5,0), 
              argos::CVector3(4,5.5,0), 
              argos::CVector3(0.3, 0.3, 0.2), 
              argos::CVector3(0.5, 0.5, 0.2), 
              whiteGoal, blueGoal, 6, 0.9999f);
}

void masterLoopFunction::PreStep() 
{

  /* Update camera objects each iteration*/
   for (size_t i = 0; i < camera::objectContainer.size(); i++)
      camera::objectContainer[i]->step();
      

   /* First iteration scan the map for boxes and update swarmManger */
   if(firstIteration)
   {
      argos::CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
      size_t i = 0;
      for(argos::CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator, i++)
      {
         argos::CBoxEntity* box = argos::any_cast<argos::CBoxEntity*>(iterator->second);

         std::string boxId = box->GetId();

         if(boxId.compare(0,3,"box") == 0) // Check that box is not a wall
            swarmMan.swarmBoxes.push_back(box);

         i++;
      }

      numBoxes = swarmMan.swarmBoxes.size();
      firstIteration = false;
   }
   
   swarmMan.step();
}




/*******************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
