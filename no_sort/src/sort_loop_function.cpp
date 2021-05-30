#include "sort_loop_function.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <sstream>
#include <list>


/****************************************/
/****************************************/

static const Real        FB_RADIUS        = 0.085036758f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.085036758f);
static const std::string FB_CONTROLLER    = "ffc";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;

/****************************************/
/****************************************/

CSortLoopFunction::CSortLoopFunction() {
}

/****************************************/
/****************************************/

CSortLoopFunction::~CSortLoopFunction() {
}

/****************************************/
/****************************************/

void CSortLoopFunction::Init(TConfigurationNode& t_tree) {
   try {
         CVector3 min_range, max_range;
         CVector3 min_orientation, max_orientation;
         CVector3 min_size, max_size;
         
         UInt32 unBox;

         TConfigurationNodeIterator itDistr;         
         /* Get current node */
         for(itDistr = itDistr.begin(&t_tree);
            itDistr != itDistr.end();
            ++itDistr) 
            {
            TConfigurationNode& tDistr = *itDistr;
            if(itDistr->Value() == "box") {
               GetNodeAttribute(tDistr, "min_range", min_range);
               GetNodeAttribute(tDistr, "max_range", max_range);
               GetNodeAttribute(tDistr, "min_orientation", min_orientation);
               GetNodeAttribute(tDistr, "max_orientation", max_orientation);
               GetNodeAttribute(tDistr, "min_size", min_size);
               GetNodeAttribute(tDistr, "max_size", max_size);
               GetNodeAttribute(tDistr, "quantity", unBox);
               PlaceBox(min_range, max_range, min_orientation, max_orientation, min_size, max_size, unBox);
            }
         }
      }  
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void CSortLoopFunction::PlaceBox( const CVector3& min_range,
               const CVector3& max_range,
               const CVector3& min_size,
               const CVector3& max_size,
               const CVector3& _whiteGoal,
               const CVector3& _blueGoal,
               UInt32 un_box,
               const double distanceThreshold){
   try {
      CBoxEntity* pcB;
      std::ostringstream cBId;

      for(size_t i = 0; i < un_box; ++i) {
         /* Make the id */
         cBId.str("");
         cBId << "box_" << i;
         
         CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
         CVector3 c_position;
         CQuaternion c_orientation_quartention;
         CVector3 c_size;

         /* Get range for position */
         CRange<double> cRangeRangeX(min_range[0], max_range[0]);
         CRange<double> cRangeRangeY(min_range[1], max_range[1]);
         CRange<double> cRangeRangeZ(min_range[2], max_range[2]);


         /* Find positions not too close too eachother according to threshold */
         argos::Real dist = 0;
         if(boxLocations.size() == 0)
         {
            c_position.SetX(pcRNG->Uniform(cRangeRangeX));
            c_position.SetY(pcRNG->Uniform(cRangeRangeY));
            c_position.SetZ(pcRNG->Uniform(cRangeRangeZ));
            boxLocations.push_back(c_position);
         }
         else
         {
            bool validLocationFound = false;
            argos::Real dist = 0.0f;
            while(!validLocationFound)
            {
               c_position.SetX(pcRNG->Uniform(cRangeRangeX));
               c_position.SetY(pcRNG->Uniform(cRangeRangeY));
               c_position.SetZ(pcRNG->Uniform(cRangeRangeZ));
               
               int count = 0;
               for(auto pos : boxLocations)
               {
                  dist = argos::Distance(pos, c_position);
                  if(dist > distanceThreshold)
                     count++;
                  else
                  {
                     break;
                  }
               }
               if(count == boxLocations.size())
               {
                  //argos::LOG << "dist: " << dist <<std::endl;
                  boxLocations.push_back(c_position);

                  dist = 0;
                  validLocationFound = true;
               }
               else
               {
                  pcRNG = CRandom::CreateRNG("argos"); //Update random seed
               }
               count = 0;
            }
         }

         /* Calculate orientation according to goal */
         argos::CRadians orientation;
         if(i%2 == 0) //Add blue box
            orientation = argos::ATan2(_blueGoal.GetX() - c_position.GetX(), _blueGoal.GetY() - c_position.GetY());
         else //Add white box
            orientation = argos::ATan2(_whiteGoal.GetX() - c_position.GetX(), _whiteGoal.GetY() - c_position.GetY());

         std::vector<argos::CRadians> box_orientation = {
            orientation + argos::ToRadians(argos::CDegrees(45)),
            argos::ToRadians(argos::CDegrees(0)), 
            argos::ToRadians(argos::CDegrees(0)) 
         };

         c_orientation_quartention.FromEulerAngles(
            box_orientation[0],
            box_orientation[1],
            box_orientation[2]
            );


         
         /* Randomize size */
         CRange<double> cRangeSizeX(min_size[0], max_size[0]);
         CRange<double> cRangeSizeY(min_size[1], max_size[1]);
         CRange<double> cRangeSizeZ(min_size[2], max_size[2]);

         c_size.SetX(pcRNG->Uniform(cRangeSizeX));
         c_size.SetY(pcRNG->Uniform(cRangeSizeY));
         c_size.SetZ(pcRNG->Uniform(cRangeSizeZ));

         /* Define new CBoxEntity */
         pcB = new CBoxEntity(
            cBId.str(),
            c_position,
            c_orientation_quartention,
            true, //moveable
            c_size,
            20); //mass
         argos::CLoopFunctions::AddEntity(*pcB);

         if(i%2 == 0) //Add blue led
         {
            pcB->AddLED(argos::CVector3(0,0,0.2), argos::CColor::BLUE);
         }
         else //Add white led
         {
            pcB->AddLED(argos::CVector3(0,0,0.2), argos::CColor::WHITE);
         }
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing boxes", ex);
   }
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSortLoopFunction, "sort_loop_function");

/****************************************/
/****************************************/
