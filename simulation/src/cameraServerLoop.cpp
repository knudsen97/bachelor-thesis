#include "../inc/cameraServerLoop.h"


//out of thread statemachine
#define DISTRIBUTE_CORNERS 0
#define JOIN_THREADS 5
#define WAIT 6
#define SEND_VELOCITY 7
#define SEND_STOP 8

//For thread statemachine
#define PLANNING 1
#define SEND_GOAL 2
#define RECEIVE 3
#define SEND_ORIENTATION 4
#define RECEIVE_STATE 5

bool testDebug = false;
cv::Mat testMap;

//debug variables
bool donePlanning = false;
int threadaState = 0;
int wasHere = 0;
bool recipocu = false;
std::vector<cv::Mat> wavefront_debug;
std::vector<cv::Mat> camera_debug;
std::vector<argos::CVector3> robot_debug, corner_debug, boxGoal_debug; 
std::vector<std::vector<cv::Point>> subgoal_debug;
std::vector<std::string> debugMessage;

std::vector<int> numPushPoints; 


/**
 * @brief This function converts a ArgOS CVector3 to an OpenCV Point.
 * @param arg An ArgOS CVector3
**/
cv::Point convertToCV(argos::CVector3 arg)
{
   return {arg.GetX()*SCALE, arg.GetY()*SCALE};
}

cameraServerLoop::cameraServerLoop(){
   /* data */
   cameraServerLoop::positionRecieved = false;
   cameraServerLoop::cornerFound = false;
   cameraServerLoop::currentState = 0;
   cameraServerLoop::threadsOpened = false;
   cameraServerLoop::allPositionRecieved = false;
   cameraServerLoop::prepareToPushDone = false;
   cameraServerLoop::stateCheck = 0;
   currentState = DISTRIBUTE_CORNERS;
}

void cameraServerLoop::operator()(int clientcount_) 
{
   clientcount = clientcount_;
   startLocations.resize(clientcount);

   //debug
   wavefront_debug.resize(clientcount);
   camera_debug.resize(clientcount);
   robot_debug.resize(clientcount);
   corner_debug.resize(clientcount);
   boxGoal_debug.resize(clientcount);
   subgoal_debug.resize(clientcount);
   debugMessage.resize(clientcount);
   numPushPoints.resize(clientcount,0);


   debug.resize(clientcount, false);
   debugMaps.resize(clientcount);
}

void cameraServerLoop::connect() 
{
   std::thread connecting(&cameraServerLoop::connect_, this);
   connecting.join();
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
            std::cout << "SERVER DIST_CORNERS\n";
            if(!threadsOpened)
            {
               /*Find where to push on the box to get to goal:*/
               std::vector<CVector3> validPushPoints;
               validPushPoints = planner::FindPushPoints(pcBox, goal);
               // std::cout << "push points: " << validPushPoints.size() << std::endl;
               // for(auto point : validPushPoints)
               //    std::cout << "point: " << point << std::endl;

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
                  robotThreads[idxPH] = std::thread(&cameraServerLoop::PrepareToPush, this, goal, 
                                       startLocations[idxPH], validPushPoints[i], threadCurrentState[idxPH], idxPH);

                  robotThreads[idxPH].detach();
                  // std::cout << "start: " << startLocations[idxPH] << std::endl;
                  // std::cout << "corner: " << validPushPoints[i] << std::endl;
                  /*Reset variables*/
                  PH = 0.0f;
                  shortestDistance = 9999.99f;
               }

               threadsOpened = true;
            }
            else
            {
               /*Debug*/
               // for (size_t i = 0; i < clientcount; i++)
               // {
               //    std::cout << "debug " << i << ": " << debug[i] << std::endl;
               //    //std::cout << "subgoals: " << subgoal_debug[i] << std::endl;
               //    // if(wavefront_debug[i].cols > 0 && !testDebug)
               //    // {
               //    //    cv::imshow("wavefront " + std::to_string(i), wavefront_debug[i]);
               //    //    cv::waitKey(1);
               //    //    if(i == clientcount - 1)
               //    //       testDebug = true;
               //    // }
               // }     

               for(auto goals : numPushPoints)
                  std::cout << "# PP: " << goals << std::endl;


               stateCheck = 0;
               for(size_t i = 0; i < clientcount; i++)
               {
                  std::cout << "client " << i << ": " << threadCurrentState[i] << std::endl;
                  if(threadCurrentState[i] == WAIT)
                  {
                     stateCheck++;
                  }
               }
               
               std::cout << "State check: " << stateCheck << std::endl;
               if(stateCheck == clientcount)
                  currentState = JOIN_THREADS;
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
            break;
         }

         /************************* JOIN_THREADS *************************/
         case JOIN_THREADS:
         {
            std::cout << "SERVER JOIN_THREADS\n";
            prepareToPushDone = true;


            currentState = SEND_VELOCITY;

            break;
         }

         /************************* SEND_VELOCITY *************************/
         case SEND_VELOCITY:
         {
            std::cout << "SERVER SEND_VELOCITY\n";
            argos::Real velocityMessage = 2.0f;
            for(int i = 0; i < clientcount; i++)
            {
               if(clientConnections[i].send(velocityMessage));
                  currentState = WAIT;
            }
            break;
         }

         /************************* WAIT *************************/
         case WAIT:
         {
            std::cout << "SERVER WAIT\n";
            argos::CVector3 boxOrigin;
            boxOrigin = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;

            argos::Real distanceToGoal = sqrt(pow(goal.GetX() - boxOrigin.GetX(), 2) + pow(goal.GetY() - boxOrigin.GetY(), 2));
            std::cout << "dist: " << distanceToGoal << std::endl;
            if(distanceToGoal < 0.049999f)
            {
               std::cout << "IN GOAL RANGE\n";
               for (size_t i = 0; i < clientcount; i++)
               {
                  if(clientConnections[i].send("STOP"));
                  argos::LOG << "----------------------send message-------------------------------- \n";
               }
               currentState = SEND_STOP;
            }
            break;
         }

         /************************* SEND_STOP *************************/
         case SEND_STOP:
         {
            std::cout << "SERVER SEND_STOP\n";
            argos::Real velocityMessage = 0.0f;
            for(int i = 0; i < clientcount; i++)
            {
               if(clientConnections[i].send(velocityMessage));
            }
            break;
         }
      }
   }
}

void cameraServerLoop::connect_()
{
   clientConnected = 0;
   serverSocket.Listen(portnumber);
   clientSockets.resize(clientcount);
   recievedPosition.resize(clientcount);
   for (size_t i = 0; i < clientcount; i++)
   {
      serverSocket.Accept(clientSockets[i]);
      clientConnections.push_back(protocol(clientSockets[i]));
      clientConnected++;
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
   while(!prepareToPushDone)
   {
      //threadCurrentState[id] = currentState;
      //threadaState = currentState;
      switch (currentState)
      {
      /************************* PLANNING *************************/
      case PLANNING:
      {
         planComplete = Planning(goal, startLoc, cornerLoc, subGoals, id);
         numPushPoints[id] = subGoals.size();
         boxGoal_debug[id] = goal;
         robot_debug[id] = startLoc;
         corner_debug[id] = cornerLoc;

         if(planComplete)
         {
            currentState = SEND_GOAL;
         }
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
            currentState = RECEIVE_STATE;
         break;
      }

      /************************* RECEIVE_STATE *************************/
      case RECEIVE_STATE:
      {
         std::string message;
         if(clientConnections[id].recieve(message))
         {
            if(message == "WAIT")
            {
               //debugMessage[id] = message;
               threadCurrentState[id] = WAIT;
            }
         }
         break;
      }
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
 
   /*Find a point on the line between corner and goal*/
   argos::CVector3 CG = planner::push(pcBox,cornerLoc, goal) - cornerLoc; //Corner-Goal vector
   CG = CG.Normalize() * (-OFF_SET);
   cornerLoc += CG;

   C.camera::GetPlot(map);
   camera_debug[id] = map.clone();

   gerymap = P.planner::Wavefront(map, startLoc, cornerLoc, debug[id], debugMaps[id]);
   wavefront_debug[id] = gerymap.clone();

   subGoals = P.planner::Pathfinder(gerymap, startLoc, cornerLoc, debug[id], debugMaps[id]);

   subgoal_debug[id] = subGoals;

   return true;
}





/*Static variables definitions*/
int cameraServerLoop::clientcount = 0;
int cameraServerLoop::portnumber = 0;

CBoxEntity* cameraServerLoop::pcBox(NULL);
CFootBotEntity* cameraServerLoop::fBot(NULL);

