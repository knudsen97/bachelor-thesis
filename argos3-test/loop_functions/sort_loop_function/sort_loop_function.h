/*
 * This example shows how to define custom distributions to place the robots.
 */

#include <argos3/core/simulator/loop_functions.h>
#include <string>

using namespace argos;

class CSortLoopFunction : public CLoopFunctions {

public:

   CSortLoopFunction();
   virtual ~CSortLoopFunction();

   virtual void Init(TConfigurationNode& t_tree);

private:
   void PlaceBox(const CVector3& min_range,
                 const CVector3& max_range,
                 const CVector3& min_orentation,
                 const CVector3& max_orientation,
                 const CVector3& min_size,
                 const CVector3& max_size,
                 const UInt32 un_box);

private:


};

