#include "cameraServerLoop.h"

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
std::vector<cv::Mat> planner_debug;
std::vector<cv::Mat> camera_debug;
std::vector<argos::CVector3> robot_debug, corner_debug, boxGoal_debug; 
std::vector<std::vector<cv::Point>> cv_subgoal_debug;
std::vector<std::vector<argos::CVector3>> subgoal_debug;
std::vector<std::string> debugMessage;

std::vector<int> numPushPoints; 


/**
 * @brief This function converts a ArgOS CVector3 to an OpenCV Point.
 * @param arg An ArgOS CVector3
**/
cv::Point convertToCV(argos::CVector3 arg)
{
   return cv::Point(arg.GetX()*SCALE, arg.GetY()*SCALE);
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
   cv_subgoal_debug.resize(clientcount);
   subgoal_debug.resize(clientcount);
   planner_debug.resize(clientcount);
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
      CVector3 boxGoal;
      boxGoal.Set(1, 1, 0);

      std::cout << "Server state: "<< currentState << std::endl;
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
               planner P;
               validPushPoints = P.FindPushPoints(pcBox, boxGoal);
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
               cameraImage = cam.GetPlot();
               
               argos::CVector3 robotEndPoint(0,0,0);
               for(auto valid : validPushPoints)
                  std::cout << "Push point" << valid << std::endl;

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

                  cameraImage = cam.GetPlot();
                  cv::circle(cameraImage, convertToCV( boxGoal ), 7, cv::Scalar(0,200,255), -1 );


                  for (size_t k = 0; k < validPushPoints.size(); k++)
                  {
                     robotEndPoint = plan.push(pcBox, validPushPoints[k], boxGoal);
                     cv::circle(cameraImage, convertToCV(robotEndPoint), 3, cv::Scalar(255,0,0), -1);

                     if (k!=i)
                     {
                        robotEndPoint = plan.push(pcBox, validPushPoints[k], boxGoal) - validPushPoints[k];
                        robotEndPoint = robotEndPoint.Normalize() * (-OFF_SET);
                        robotEndPoint += validPushPoints[k];
                        //cv::circle(cameraImage, convertToCV(robotEndPoint), 7, cv::Scalar(255,0,0), -1);
                     }
                  }

                  /* Define a kernel and erode the map in order to not get close to obstacles */
                  int dilation_size = 0.15*SCALE;
                  cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                     cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                     cv::Point( dilation_size, dilation_size ) );
                  cv::erode(cameraImage, cameraImage, kernel);

                  /* Planning */
                  std::vector<cv::Point> subGoals;
                  bool planComplete = Planning(cameraImage, boxGoal, startLocations[idxPH], validPushPoints[i], subGoals);

                  /* Visual aid */
                  for(auto point : subGoals)
                     cv::circle(cameraImage, point, 5, cv::Scalar(0,200,200), -1);
                  cv::circle(cameraImage, cv::Point(startLocations[idxPH].GetX()*SCALE, startLocations[idxPH].GetY()*SCALE) , 5, cv::Scalar(0,0,255), -1);
                  cv::circle(cameraImage, subGoals.back(), 5, cv::Scalar(0,255,0), -1);

                  cv::imshow("map", cameraImage);
                  cv::waitKey(0);

                  /*Start thread*/
                  // robotThreads[idxPH] = std::thread(&cameraServerLoop::PrepareToPush, this, cameraImage, boxGoal, 
                  //                      startLocations[idxPH], validPushPoints[i], threadCurrentState[idxPH], idxPH);
                  std::cout << "subGoals: " << subGoals.size() << std::endl;
                  robotThreads[idxPH] = std::thread(&cameraServerLoop::PrepareToPush, this, boxGoal, 
                     subGoals, threadCurrentState[idxPH], idxPH);

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
               // /*Debug*/
               // for (size_t i = 0; i < clientcount; i++)
               // {
               //    std::cout << "id: " << i << '\n';
               //    std::cout << "argos subgoal: \n";
               //    for (size_t j = 0; j < subgoal_debug[i].size(); j++)
               //    {
               //       std::cout << "....." << subgoal_debug[i][j] << "\n";
               //    }
               //    std::cout << "\n";
                  
               //    std::cout << "cv subgoal: \n";
               //    for (size_t j = 0; j < cv_subgoal_debug[i].size(); j++)
               //    {
               //       std::cout << "....." << cv_subgoal_debug[i][j].x << " " << cv_subgoal_debug[i][j].y << "\n";
               //    }
               //    std::cout << "\n";
               // }     

               // for(auto goals : numPushPoints)
               //    std::cout << "# PP: " << goals << std::endl;

               stateCheck = 0;
               for(size_t i = 0; i < clientcount; i++)
               {
                  // std::cout << "client " << i << ": " << threadCurrentState[i] << std::endl;
                  if(threadCurrentState[i] == WAIT)
                  {
                     stateCheck++;
                  }
               }
               
               // std::cout << "State check: " << stateCheck << std::endl;
               if(stateCheck == clientcount)
                  currentState = JOIN_THREADS;
            }
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
               {
                  currentState = WAIT;
                  std::cout << "test: " << clientConnections.size() << std::endl;
               }
            }
            break;
         }

         /************************* WAIT *************************/
         case WAIT:
         {
            std::cout << "SERVER WAIT\n";
            argos::CVector3 boxOrigin;
            boxOrigin = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;

            argos::Real distanceToGoal = sqrt(pow(boxGoal.GetX() - boxOrigin.GetX(), 2) + pow(boxGoal.GetY() - boxOrigin.GetY(), 2));
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
// void cameraServerLoop::PrepareToPush(cv::Mat map, argos::CVector3 goal, argos::CVector3 startLoc, 
//                                        argos::CVector3 cornerLoc, int currentState_, int id)
void cameraServerLoop::PrepareToPush(argos::CVector3 boxGoal, std::vector<cv::Point> subGoals, int currentState_, int id)
{  
   int currentState = currentState_;
   //std::vector<cv::Point> subGoals;
   bool planComplete = true;
   int curGoal = 0;
   int time;
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
         //planComplete = Planning(map, goal, startLoc, cornerLoc, subGoals, id);

         /*Debugging*/
         // numPushPoints[id] = subGoals.size();
         // boxGoal_debug[id] = goal;
         // robot_debug[id] = startLoc;
         // corner_debug[id] = cornerLoc;
         // subgoal_debug[id].resize(subGoals.size());
         // for (size_t i = 0; i < subGoals.size(); i++)
         // {
         //    subgoal_debug[id][i] = argos::CVector3(subGoals[i].x/(double)SCALE, subGoals[i].y/(double)SCALE, 0);
         // }
         

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
//         boxGoal_debug[id] = goalPoint;
         if(clientConnections[id].send(goalPoint))
            currentState = RECEIVE;
         time = std::time(NULL);
      }
      break;

      /************************* RECEIVE *************************/
      case RECEIVE:
      {
         argos::Real message;
         if ((std::time(NULL) - time) > 2)
         {
            currentState = SEND_GOAL;
            wasHere++;
         }
         
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
         argos::CVector3 g = plan.push(pcBox, robotPosition, boxGoal);
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
bool cameraServerLoop::Planning(cv::Mat &map_, argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, std::vector<cv::Point> &subGoals)//, int id)
{
   planner P;
   cv::Mat grayMap;
   //cv::Mat map = map_.clone();
 
   /*Find a point on the line between corner and goal*/
   argos::CVector3 CG = P.push(pcBox,cornerLoc, goal) - cornerLoc; //Corner-Goal vector
   CG = CG.Normalize() * (-OFF_SET);
   cornerLoc += CG;

   grayMap = P.planner::Wavefront(map_, startLoc, cornerLoc);

   subGoals = P.planner::Pathfinder(grayMap, startLoc, cornerLoc);
   //cv_subgoal_debug[id] = subGoals;

   return true;
}





/*Static variables definitions*/
int cameraServerLoop::clientcount = 0;
int cameraServerLoop::portnumber = 0;

CBoxEntity* cameraServerLoop::pcBox(NULL);
CFootBotEntity* cameraServerLoop::fBot(NULL);

