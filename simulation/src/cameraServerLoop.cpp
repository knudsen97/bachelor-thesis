#include "../inc/cameraServerLoop.h"

#define PLANNING 0
#define SEND_GOAL 1
#define RECEIVE 2
#define SEND_ORIENTATION 3
#define WAIT 4

void cameraServerLoop::init() 
{
   std::thread connecting(connect);
   connecting.join();
   currentState = PLANNING;
}

void cameraServerLoop::step()
{
   CVector3 goal;
   goal.Set(2, 2, 0);

   /************************* FSM START *************************/
   switch (currentState)
   {
   /************************* PLANNING *************************/
   case PLANNING:
   {
      std::cout << "SERVER PLANNING\n";
      planComplete = Planning(goal);
      if(planComplete)
         currentState = SEND_GOAL;
   }
   break;

   /************************* SEND GOAL *************************/
   case SEND_GOAL:
   {
      std::cout << "SERVER SEND_GOAL\n";
      argos::CVector3 goalPoint = argos::CVector3(subGoals[i].x/(double)SCALE, subGoals[i].y/(double)SCALE, 0);   
      
      if(clientConnections[0].send(goalPoint))
         currentState = RECEIVE;
   }
   break;

   /************************* RECEIVE *************************/
   case RECEIVE:
   {
      std::cout << "SERVER RECEIVE\n";

      argos::Real message;
      
      
      if(clientConnections[0].recieve(robotPosition))
      {
         std::cout << robotPosition << std::endl;
         i++;
         std::cout << "Current i: " << i << std::endl;

         if(i < subGoals.size())
            currentState = SEND_GOAL;
         else
            currentState = SEND_ORIENTATION;
      }
   }
   break;

   /************************* SEND_ORIENTATION *************************/
   case SEND_ORIENTATION:
   {
      std::cout << "SERVER SEND_ORIENTATION\n";
      argos::CRadians goalAngle = argos::ATan2(goal.GetY()-robotPosition.GetY(), goal.GetX()-robotPosition.GetX());
      if(clientConnections[0].send(goalAngle, argos::CRadians(0), argos::CRadians(0)))
         currentState = WAIT;
   }
   break;

   /************************* WAIT *************************/
   case WAIT:
   {
      std::cout << "SERVER WAIT\n";
   }
   break;

   }

}

void cameraServerLoop::connect()
{
   serverSocket.Listen(portnumber);
   clientSockets.resize(clientcount);
   for (size_t i = 0; i < clientcount; i++)
   {
      serverSocket.Accept(clientSockets[i]);
      clientConnections.push_back(protocol(clientSockets[i]));
   }
}


/**
 * A function to call the planning algorithms such as wavefront and pathfinder
 * @param goal The goal position
*/
bool cameraServerLoop::Planning(argos::CVector3 &goal)
{
   planner P;
   //Find where to push on the box to get to goal:
   std::vector<CVector3> validPushPoints;
   validPushPoints = P.FindPushPoints(pcBox, goal);

   CVector3 goalLoc, startLoc;
   startLoc = startLocations[0]; //startLocations found in masterLoopFunction
   //Assign a corner to do wavefront/pathfinder on:
   goalLoc = validPushPoints[0];

   //Find a point on the line between corner and goal
   argos::CVector3 CG = goal - goalLoc; //Corner-Goal vector
   CG = CG.Normalize() * (-OFF_SET);
   goalLoc += CG;

   //for(argos::CVector3 s : )
   C.camera::GetPlot(map);

   map = P.planner::Wavefront(map, startLoc, goalLoc);
   subGoals = P.planner::Pathfinder(map, startLoc, goalLoc);
   std::cout << "subgoals size: " << subGoals.size() << std::endl;
   return true;
}

int cameraServerLoop::clientcount = 0;
int cameraServerLoop::portnumber = 0;
argos::CTCPSocket cameraServerLoop::serverSocket;
std::vector<argos::CTCPSocket> cameraServerLoop::clientSockets;
std::vector<argos::CVector3> cameraServerLoop::robotPositions;
std::vector<protocol> cameraServerLoop::clientConnections;
bool cameraServerLoop::positionRecieved = false;

bool cameraServerLoop::planComplete = false;
CBoxEntity* cameraServerLoop::pcBox(NULL);
CFootBotEntity* cameraServerLoop::fBot(NULL);

std::vector<CVector3> cameraServerLoop::startLocations;
cv::Mat cameraServerLoop::map;
std::vector<cv::Point> cameraServerLoop::subGoals;
int cameraServerLoop::i = 0;
bool cameraServerLoop::cornerFound = false;

camera cameraServerLoop::C;

int cameraServerLoop::currentState = 0;

argos::CVector3 cameraServerLoop::robotPosition;



