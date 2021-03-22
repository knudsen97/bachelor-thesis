#include "../inc/masterLoopFunction.h"


masterLoopFunction::masterLoopFunction(/* args */) 
{
    
}

masterLoopFunction::~masterLoopFunction() 
{
    
}

void masterLoopFunction::Init(argos::TConfigurationNode& t_tree) {

   
   try {
         TConfigurationNodeIterator itDistr;         
         /* Get current node */
         for(itDistr = itDistr.begin(&t_tree);
            itDistr != itDistr.end();
            ++itDistr) 
            {
            TConfigurationNode& tDistr = *itDistr;
            if(itDistr->Value() == "server") {
                
                GetNodeAttribute(tDistr, "port", cameraServerLoop::portnumber);
                GetNodeAttribute(tDistr, "clients", cameraServerLoop::clientcount);
            }
         }
      }  
   catch(argos::CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
   //std::cout << "portnumber: " << cameraServerLoop::portnumber << std::endl;

   cameraServerLoop::init();
   //std::cout << "Finish master init" << std::endl;

}

void masterLoopFunction::PreStep() 
{
   //std::cout << "Loopfunction PreStep" << std::endl;
   //std::cout << "client count: " << cameraServerLoop::clientcount << std::endl;   
   for (size_t i = 0; i < camera::objectContainer.size(); i++)
      camera::objectContainer[i]->step();
      
   CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
   for (CSpace::TMapPerType::iterator iterator = boxMap.begin(); iterator != boxMap.end(); ++iterator)
   {   
      pcBox = any_cast<CBoxEntity*>(iterator->second);
      if (pcBox->GetId() == "box1")
         break;
   }

   cameraServerLoop::step();

      
}



/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(masterLoopFunction, "master_loop_function");

/****************************************/
/****************************************/
