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
               const CVector3& min_orientation,
               const CVector3& max_orientation,
               const CVector3& min_size,
               const CVector3& max_size,
               UInt32 un_box){
   try {
      CBoxEntity* pcB;
      std::ostringstream cBId;

      for(size_t i = 0; i < un_box; ++i) {
         /* Make the id */
         cBId.str("");
         cBId << "box" << i;
         
         CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
         CVector3 c_position;
         CQuaternion c_orientation_quartention;
         CVector3 c_size;

         //convert angle from degree to radian
         std::vector<argos::CDegrees> min_orientation_degree = {argos::CDegrees(min_orientation.GetX()), argos::CDegrees(min_orientation.GetY()), argos::CDegrees(min_orientation.GetZ())};
         std::vector<argos::CRadians> min_orientation_radian = {argos::ToRadians(min_orientation_degree[0]), argos::ToRadians(min_orientation_degree[1]), argos::ToRadians(min_orientation_degree[2])};

         std::vector<argos::CRadians> max_orientation_radian = {
            argos::ToRadians(argos::CDegrees(max_orientation.GetX())), 
            argos::ToRadians(argos::CDegrees(max_orientation.GetY())), 
            argos::ToRadians(argos::CDegrees(max_orientation.GetZ()))
         };

         //randomize the orientation
         CRange<CRadians> cRangeOrientationX( min_orientation_radian[0],max_orientation_radian[0] );
         CRange<CRadians> cRangeOrientationY( min_orientation_radian[1],max_orientation_radian[1] );
         CRange<CRadians> cRangeOrientationZ( min_orientation_radian[2],max_orientation_radian[2] );

         c_orientation_quartention.FromEulerAngles(
            pcRNG->Uniform(cRangeOrientationZ),
            pcRNG->Uniform(cRangeOrientationY), 
            pcRNG->Uniform(cRangeOrientationX) 
            );

         //randomize position
         CRange<double> cRangeRangeX(min_range[0], max_range[0]);
         CRange<double> cRangeRangeY(min_range[1], max_range[1]);
         CRange<double> cRangeRangeZ(min_range[2], max_range[2]);

         c_position.SetX(pcRNG->Uniform(cRangeRangeX));
         c_position.SetY(pcRNG->Uniform(cRangeRangeY));
         c_position.SetZ(pcRNG->Uniform(cRangeRangeZ));

         //randomize size
         CRange<double> cRangeSizeX(min_size[0], max_size[0]);
         CRange<double> cRangeSizeY(min_size[1], max_size[1]);
         CRange<double> cRangeSizeZ(min_size[2], max_size[2]);

         c_size.SetX(pcRNG->Uniform(cRangeSizeX));
         c_size.SetY(pcRNG->Uniform(cRangeSizeY));
         c_size.SetZ(pcRNG->Uniform(cRangeSizeZ));

         pcB = new CBoxEntity(
            cBId.str(),
            c_position,
            c_orientation_quartention,
            true, //moveable
            c_size,
            20); //mass
         argos::CLoopFunctions::AddEntity(*pcB);
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
