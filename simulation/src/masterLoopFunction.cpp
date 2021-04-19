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

   argos::CBoxEntity* pcBox;
   //std::cout << "Loopfunction PreStep" << std::endl;
   //std::cout << "client count: " << cameraServerLoop::clientcount << std::endl;   
   for (size_t i = 0; i < camera::objectContainer.size(); i++)
      camera::objectContainer[i]->step();
      
   argos::CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
   
   size_t i = 0;
   for(argos::CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator, i++)
   {
      argos::CBoxEntity* box = argos::any_cast<argos::CBoxEntity*>(iterator->second);

      std::string boxId = box->GetId();
      if(firstIteration)
      {
         if(boxId.compare(0,3,"box") == 0)
            swarmMan.swarmBoxes.push_back(box);
      }
      else
      {
         if(boxId.compare(0,3,"box") == 0)
            swarmMan.swarmBoxes[i] = box;
      }
      i++;
   }
   if(firstIteration)
   {
      numBoxes = swarmMan.swarmBoxes.size();
      //server(footbotCount, {2,2,0}, swarmMan.swarmBoxes[0]);
      firstIteration = false;
   }

   //pcBox = swarmMan.swarmBoxes[0];
   //server.step(); //Den crasher programmet
   swarmMan.step();
   //server.step();
}




/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
