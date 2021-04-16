#include "../inc/masterLoopFunction.h"


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

   int footbotCount = 0;
   argos::CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("foot-bot");
   for (argos::CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
      footbotCount++;

   server(footbotCount);
   server.connect();
   //std::cout << "Finish master init" << std::endl;

}

void masterLoopFunction::PreStep() 
{
   //std::cout << "Loopfunction PreStep" << std::endl;
   //std::cout << "client count: " << cameraServerLoop::clientcount << std::endl;   
   for (size_t i = 0; i < camera::objectContainer.size(); i++)
      camera::objectContainer[i]->step();
      
   argos::CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
   for (argos::CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator)
   {   
      pcBox = argos::any_cast<argos::CBoxEntity*>(iterator->second);
      if (pcBox->GetId() == "box1")
         break;
   }

   server.step();

      
}



/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
