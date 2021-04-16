#include "../inc/masterLoopFunction.h"
bool firstRun = true;
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


   //std::cout << "Finish master init" << std::endl;

}

void masterLoopFunction::PreStep() 
{

   argos::CBoxEntity* pcBox;
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
   if (firstRun)
   {
      server(footbotCount, {2,2,0}, pcBox);
      server.connect();
      firstRun = false;
   }

   server.step();

      
}



/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
