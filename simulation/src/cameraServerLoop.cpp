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
std::vector<cv::Mat> wavefront_debug;
std::vector<cv::Mat> camera_debug;
std::vector<argos::CVector3> robot_debug, corner_debug, boxGoal_debug; 
std::vector<std::vector<cv::Point>> subgoal_debug;

/**
 * @brief This function converts a ArgOS CVector3 to an OpenCV Point.
 * @param arg An ArgOS CVector3
**/
cv::Point convertToCV(argos::CVector3 arg)
{
   return {arg.GetX()*SCALE, arg.GetY()*SCALE};
}

void cameraServerLoop::init() 
{
   std::thread connecting(connect);
   connecting.join();
   currentState = DISTRIBUTE_CORNERS;
   startLocations.resize(clientcount);

   //debug
   wavefront_debug.resize(clientcount);
   camera_debug.resize(clientcount);
   robot_debug.resize(clientcount);
   corner_debug.resize(clientcount);
   boxGoal_debug.resize(clientcount);
   subgoal_debug.resize(clientcount);
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

      /************************* FSM START *************************/
      switch (currentState)
      {
         /************************* DISTRIBUTE_CORNERS *************************/
         case DISTRIBUTE_CORNERS:
         { 
            if(!threadsOpened)
            {
               /*Find where to push on the box to get to goal:*/
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
                     PH = abs(startLocations[j].GetX() - validPushPoints[i].GetX() 
                            + startLocations[j].GetY() - validPushPoints[i].GetY());

                     if(PH < shortestDistance && isRobotAssigned[j] == false)
                     {
                        shortestDistance = PH;
                        idxPH = j;
                     }
                  }
                  isRobotAssigned[idxPH] = true;
                  std::cout << "indexPH: " << idxPH << std::endl;

                  /*Start thread*/
                  robotThreads[idxPH] = std::thread(&cameraServerLoop::PrepareToPush, goal, 
                                       startLocations[idxPH], validPushPoints[i], threadCurrentState[idxPH], idxPH);
                  robotThreads[idxPH].detach();
                  std::cout << "start: " << startLocations[idxPH] << std::endl;
                  std::cout << "corner: " << validPushPoints[i] << std::endl;
                  /*Reset variables*/
                  PH = 0.0f;
                  shortestDistance = 9999.99f;
               }

               threadsOpened = true;
            }
            // for (size_t i = 0; i < clientcount; i++)
            // {
            //    if (wavefront_debug[i].cols > 0)
            //    {
            //       cv::Mat robots = camera_debug[i].clone();
            //       // std::string name = "wavefront: " + std::to_string(i);
            //       // std::string name2 = "camera: " + std::to_string(i);
            //       // cv::imshow(name, wavefront_debug[i]);

            //       // cv::imshow(name2, camera_debug[i]);

            //       argos::LOG << "robot: " << std::to_string(i) << '\n';
            //       argos::LOG << "boxGoal" << boxGoal_debug[i] << '\n';
            //       argos::LOG << "robot" << robot_debug[i] << '\n';
            //       argos::LOG << "corner" << corner_debug[i] << '\n';
            //       argos::LOG << '\n'; 


            //    }               
            // }
            // if (wavefront_debug[0].cols > 0)
            // {
            //    cv::Mat robots = camera_debug[0];
            //    cv::circle(robots, convertToCV(boxGoal_debug[0]), 2, {0,0,255}, -1);

            //    //draw robot
            //    cv::circle(robots, convertToCV(robot_debug[0]), 4, {0,0,255}, -1);
            //    cv::circle(robots, convertToCV(robot_debug[1]), 4, {0,255,0}, -1);
            //    cv::circle(robots, convertToCV(robot_debug[2]), 4, {255,0,0}, -1);
               
            //    //draw robot subgoals

            //    for (auto subgoal : subgoal_debug[0])
            //       cv::circle(robots, subgoal, 4, {0,0,255}, 0);
            //    for (auto subgoal : subgoal_debug[1])
            //       cv::circle(robots, subgoal, 4, {0,255,0}, 0);
            //    for (auto subgoal : subgoal_debug[2])
            //       cv::circle(robots, subgoal, 4, {255,0,0}, 0);

            //    //draw corner
            //    cv::circle(robots, convertToCV(corner_debug[0]), 6, {0,0,255}, -1);
            //    cv::circle(robots, convertToCV(corner_debug[1]), 6, {0,255,0}, -1);
            //    cv::circle(robots, convertToCV(corner_debug[2]), 6, {255,0,0}, -1);

            //    cv::imshow("debug", robots);
            // }
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

/**
 * @brief This function makes the robot go to the desired corner location and wait there.
 * @param goal The final goal location for the box
 * @param startLoc The robots start position
 * @param cornerLoc The corner location the robot needs to go to
 * @param currentState The state the robot starts in, in the state machine
 * @param id The id of the robot to keep track of them since they are run in a thread
 **/
void cameraServerLoop::PrepareToPush(argos::CVector3 goal, argos::CVector3 startLoc, 
                                       argos::CVector3 cornerLoc, int currentState_, int id)
{  
   int currentState = currentState_;
   std::vector<cv::Point> subGoals;
   bool planComplete = false;
   int curGoal = 0;
   argos::CVector3 robotPosition;
   while(true)
   {
      threadaState = currentState;
      switch (currentState)
      {
      /************************* PLANNING *************************/
      case PLANNING:
      {
         planComplete = Planning(goal, startLoc, cornerLoc, subGoals, id);
         boxGoal_debug[id] = goal;
         robot_debug[id] = startLoc;
         corner_debug[id] = cornerLoc;

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
         
         if(clientConnections[id].recieve(robotPosition))
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
         argos::CVector3 g = planner::push(pcBox, robotPosition, goal);
         argos::CRadians goalAngle = argos::ATan2(g.GetY()-robotPosition.GetY(), g.GetX()-robotPosition.GetX());
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
 * @brief A function to call the planning algorithms such as wavefront and pathfinder
 * @param goal The goal position
 * @param startLoc The start location of the robot
 * @param cornerLoc The corner location the robot need to go to
 * @param subGoals The sub goals leading up to the corner location
 * @param id The id of the robot to keep track of them since they are run in a thread
*/
bool cameraServerLoop::Planning(argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, std::vector<cv::Point> &subGoals, int id)
{
   planner P;
   cv::Mat map;
   cv::Mat gerymap;
 
   //Find a point on the line between corner and goal
   argos::CVector3 CG = planner::push(pcBox,cornerLoc, goal) - cornerLoc; //Corner-Goal vector
   CG = CG.Normalize() * (-OFF_SET);
   cornerLoc += CG;

   //for(argos::CVector3 s : )
   C.camera::GetPlot(map);
   camera_debug[id] = map.clone();

   gerymap = P.planner::Wavefront(map, startLoc, cornerLoc);
   wavefront_debug[id] = gerymap.clone();
   
   subGoals = P.planner::Pathfinder(gerymap, startLoc, cornerLoc);
   subgoal_debug[id] = subGoals;

   //std::cout << "subgoals size: " << subGoals.size() << std::endl;
   return true;
}


/*Static variables definitions*/
int cameraServerLoop::clientcount = 0;
int cameraServerLoop::portnumber = 0;
argos::CTCPSocket cameraServerLoop::serverSocket;
std::vector<argos::CTCPSocket> cameraServerLoop::clientSockets;
std::vector<argos::CVector3> cameraServerLoop::robotPositions;
std::vector<protocol> cameraServerLoop::clientConnections;
bool cameraServerLoop::positionRecieved = false;

//bool cameraServerLoop::planComplete = false;
CBoxEntity* cameraServerLoop::pcBox(NULL);
CFootBotEntity* cameraServerLoop::fBot(NULL);

std::vector<CVector3> cameraServerLoop::startLocations;
//std::vector<cv::Point> cameraServerLoop::subGoals;
bool cameraServerLoop::cornerFound = false;

camera cameraServerLoop::C;

int cameraServerLoop::currentState = 0;



std::vector<int> cameraServerLoop::threadCurrentState;

bool cameraServerLoop::threadsOpened = false;

std::vector<bool> cameraServerLoop::recievedPosition;
bool cameraServerLoop::allPositionRecieved = false;


