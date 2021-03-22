#include "../inc/cameraServerLoop.h"

//out of thread statemachine
#define DISTRIBUTE_CORNERS 0
#define JOIN_THREADS 5
#define WAIT 6

//For thread statemachine
#define PLANNING 1
#define SEND_GOAL 2
#define RECEIVE 3
#define SEND_ORIENTATION 4

//debug variables
bool donePlanning = false;
int threadaState = 0;
int wasHere = 0;
bool recipocu = false;

void cameraServerLoop::init() 
{
   std::thread connecting(connect);
   connecting.join();
   currentState = DISTRIBUTE_CORNERS;
}

void cameraServerLoop::step()
{
   //get robot positions
   if (!allPositionRecieved)
   {
      allPositionRecieved = true;
      for (size_t i = 0; i < clientcount; i++)
      {
         if (!recievedPosition[i])
         {
            CVector3 position;
            recievedPosition[i] = clientConnections[i].recieve(position);
            startLocations[i] = position;

         }
         allPositionRecieved &= recievedPosition[i];  
         argos::LOG << "recievedPosition " << i << " :" << recievedPosition[i] << '\n';
      }
   }
   else
   {
      CVector3 goal;
      goal.Set(2, 2, 0);

      std::vector<std::thread> robotThreads;

      /************************* FSM START *************************/
      switch (currentState)
      {
         /************************* DISTRIBUTE_CORNERS *************************/
         case DISTRIBUTE_CORNERS:
         { 
            if(!threadsOpened)
            {
               //Find where to push on the box to get to goal:
               std::vector<CVector3> validPushPoints;
               validPushPoints = planner::FindPushPoints(pcBox, goal);

               std::vector<std::thread> robotThreads;
               robotThreads.resize(startLocations.size());
               threadCurrentState.resize(startLocations.size(), PLANNING);

               std::vector<bool> isRobotAssigned;
               isRobotAssigned.resize(startLocations.size(), false);

               /*Find absolute distance between point and robot and assign shortest distance to each robot*/
               double shortestDistance = 9999.99f, PH = 0.0f;
               int idxPH = 0;
               for(auto valid : validPushPoints)
                  std::cout << "Push point" <<valid << std::endl;

                        
               for(size_t i = 0; i < validPushPoints.size(); i++)
               {
                  for(size_t j = 0; j < startLocations.size(); j++)
                  {
                     PH = abs(pow(startLocations[j].GetX() - validPushPoints[i].GetX(), 2) 
                           + pow(startLocations[j].GetY() - validPushPoints[i].GetY(), 2));

                     if(PH < shortestDistance && isRobotAssigned[j] == false)
                     {
                        shortestDistance = PH;
                        idxPH = j;
                     }
                  }
                  isRobotAssigned[idxPH] = true;
                  std::cout << "indexPH: " << idxPH << std::endl;

                  /*Start thread*/
                  std::cout << "i: " << std::endl;
                  robotThreads[i] = std::thread(&cameraServerLoop::PrepareToPush, goal, 
                                       startLocations[idxPH], validPushPoints[i], threadCurrentState[i], idxPH);
                  robotThreads[i].detach();
                  
                  /*Reset variables*/
                  PH = 0.0f;
                  shortestDistance = 9999.99f;
               }

               threadsOpened = true;
            }
         }
         break;


         /************************* JOIN_THREADS *************************/
         case JOIN_THREADS:
         {
            std::cout << "SERVER JOIN_THREADS\n";
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
}

void cameraServerLoop::connect()
{
   serverSocket.Listen(portnumber);
   clientSockets.resize(clientcount);
   recievedPosition.resize(clientcount);
   for (size_t i = 0; i < clientcount; i++)
   {
      serverSocket.Accept(clientSockets[i]);
      clientConnections.push_back(protocol(clientSockets[i]));
   }
}

void cameraServerLoop::PrepareToPush(argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, int currentState_, int id)
{  
   int currentState = currentState_;
   while(true)
   {
      threadaState = currentState;
      switch (currentState)
      {
      /************************* PLANNING *************************/
      case PLANNING:
      {
         planComplete = Planning(goal, startLoc, cornerLoc);
         donePlanning = planComplete;
         if(planComplete)
            currentState = SEND_GOAL;
      }
      break;

      /************************* SEND GOAL *************************/
      case SEND_GOAL:
      {
         argos::CVector3 goalPoint = argos::CVector3(subGoals[curGoal].x/(double)SCALE, subGoals[curGoal].y/(double)SCALE, 0);   
         
         if(clientConnections[id].send(goalPoint))
            currentState = RECEIVE;
         wasHere++;
      }
      break;

      /************************* RECEIVE *************************/
      case RECEIVE:
      {

         argos::Real message;
         
         if(clientConnections[0].recieve(robotPosition))
         {
            //std::cout << robotPosition << std::endl;
            curGoal++;
            recipocu = true;
            if(curGoal < subGoals.size())
               currentState = SEND_GOAL;
            else
               currentState = SEND_ORIENTATION;
         }
      }
      break;

      /************************* SEND_ORIENTATION *************************/
      case SEND_ORIENTATION:
      {
         argos::CRadians goalAngle = argos::ATan2(goal.GetY()-robotPosition.GetY(), goal.GetX()-robotPosition.GetX());
         if(clientConnections[id].send(goalAngle, argos::CRadians(0), argos::CRadians(0)))
            currentState = 10;
         break;
      }
      case 10:
      break;
      }
   }

}


/**
 * A function to call the planning algorithms such as wavefront and pathfinder
 * @param goal The goal position
*/
bool cameraServerLoop::Planning(argos::CVector3 &goal, argos::CVector3 &startLoc, argos::CVector3 &cornerLoc)
{
   planner P;
   // //Find where to push on the box to get to goal:
   // std::vector<CVector3> validPushPoints;
   // validPushPoints = P.FindPushPoints(pcBox, goal);

   // CVector3 goalLoc, startLoc;
   // startLoc = startLocations[0]; //startLocations found in masterLoopFunction
   // //Assign a corner to do wavefront/pathfinder on:
   // goalLoc = validPushPoints[0];

   //Find a point on the line between corner and goal
   argos::CVector3 CG = goal - cornerLoc; //Corner-Goal vector
   CG = CG.Normalize() * (-OFF_SET);
   cornerLoc += CG;

   //for(argos::CVector3 s : )
   C.camera::GetPlot(map);

   map = P.planner::Wavefront(map, startLoc, cornerLoc);
   subGoals = P.planner::Pathfinder(map, startLoc, cornerLoc);
   //std::cout << "subgoals size: " << subGoals.size() << std::endl;
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
int cameraServerLoop::curGoal = 0;
bool cameraServerLoop::cornerFound = false;

camera cameraServerLoop::C;

int cameraServerLoop::currentState = 0;

argos::CVector3 cameraServerLoop::robotPosition;

std::vector<int> cameraServerLoop::threadCurrentState;

bool cameraServerLoop::threadsOpened = false;

std::vector<bool> cameraServerLoop::recievedPosition;
bool cameraServerLoop::allPositionRecieved = false;
