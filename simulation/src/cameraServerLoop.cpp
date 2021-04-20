#include "cameraServerLoop.h"
#include "serverHelperFunctions.h"
#include "masterLoopFunction.h"

//out of thread statemachine
#define DISTRIBUTE_CORNERS 0
#define JOIN_THREADS 5
#define WAIT 6
#define SEND_VELOCITY 7
#define SEND_STOP 8
#define DONE 9

//For thread statemachine
#define PLANNING 1
#define SEND_GOAL 2
#define RECEIVE 3
#define SEND_ORIENTATION 4
#define RECEIVE_STATE 5


//debug variables
bool testDebug = false;
cv::Mat testMap;
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

void debugFun(int _clientcount, bool a = true)
{
   if(a)
   {
      for (size_t i = 0; i < _clientcount; i++)
      {
         std::cout << "id: " << i << '\n';
         std::cout << "argos subgoal: \n";
         for (size_t j = 0; j < subgoal_debug[i].size(); j++)
         {
            std::cout << "....." << subgoal_debug[i][j] << "\n";
         }
         std::cout << "\n";
         
         std::cout << "cv subgoal: \n";
         for (size_t j = 0; j < cv_subgoal_debug[i].size(); j++)
         {
            std::cout << "....." << cv_subgoal_debug[i][j].x << " " << cv_subgoal_debug[i][j].y << "\n";
         }
         std::cout << "\n";
      }  

      for(auto goals : numPushPoints)
         std::cout << "# PP: " << goals << std::endl;
   }
}



cameraServerLoop::cameraServerLoop(){
}

cameraServerLoop::~cameraServerLoop() 
{
}

void cameraServerLoop::operator()(int clientcount_, argos::CVector3 boxGoal_, argos::CBoxEntity* pcBox_)
{
   argos::LOG << "---------------------reseb serber------------------------\n";
   //reset
   cameraServerLoop::currentState = 0;
   cameraServerLoop::threadsOpened = false;
   cameraServerLoop::allPositionRecieved = false;
   cameraServerLoop::prepareToPushDone = false;
   cameraServerLoop::connected = false;
   cameraServerLoop::currentState = DISTRIBUTE_CORNERS;
   cameraServerLoop::stopSent = true;
   cameraServerLoop::stopSent_ = false;
   cameraServerLoop::rewind = true;
   cameraServerLoop::rewind_ = false;
   cameraServerLoop::footbotStopped = true;
   cameraServerLoop::footbotStopped_ = false;
   cameraServerLoop::inRange_ = false;
   cameraServerLoop::jobsDone = false;
   cameraServerLoop::threadClosed.clear();
   cameraServerLoop::threadCurrentState.clear();
   cameraServerLoop::recievedPosition.clear();                           


   if (clientcount != clientcount_)
   {
      if (clientcount > 0)
      {
         serverSocket.Disconnect();
      }
      
      clientcount = clientcount_;
      connect();
   }
   else
   {
      connected = true;
   }
   

   //assign variables
   boxGoal = boxGoal_;
   pcBox = pcBox_;
   startLocations.resize(clientcount);
   recievedPosition.resize(clientcount, false);

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
   connecting.detach();
}



void cameraServerLoop::step()
{
   if (connected)
   {
      //get robot positions
      if (!allPositionRecieved)
      {
         argos::LOG << "UPDATE POSITION: " << std::endl;
         allPositionRecieved = true;

         for (size_t i = 0; i < clientcount; i++)
         {
            if (!recievedPosition[i])
            {
               argos::CVector3 position;
               recievedPosition[i] = clientConnections[i].recieve(position);
               startLocations[i] = position;
               argos::LOG << position << std::endl;
            }
            allPositionRecieved &= recievedPosition[i];  
            argos::LOG << "recievedPosition " << i << " :" << recievedPosition[i] << '\n';
         }
      }
      else
      {

         argos::LOG << "Server state: "<< currentState << std::endl;
         /************************* FSM START *************************/
         switch (currentState)
         {
            /************************* DISTRIBUTE_CORNERS *************************/
            case DISTRIBUTE_CORNERS:
            { 
               argos::LOG << "SERVER DIST_CORNERS\n";
               if(!threadsOpened)
               {
                  /* Define containers */
                  std::vector<argos::CVector3> validPushPoints;
                  std::vector<std::thread> robotThreads;
                  std::vector<bool> isRobotAssigned;

                  /* Find where to push on the box to get to goal */
                  validPushPoints = plan.FindPushPoints(pcBox, boxGoal);
                  argos::LOG << "push points: " << validPushPoints.size() << std::endl;
                  for(auto point : validPushPoints)
                     argos::LOG << "point: " << point << std::endl;

                  robotThreads.resize(startLocations.size());
                  threadCurrentState.resize(startLocations.size(), PLANNING);
                  isRobotAssigned.resize(startLocations.size(), false);
                  threadClosed.resize(startLocations.size(), false);

                  for(auto threadState : threadCurrentState)
                     threadState = 1;
                  for(auto threadState : threadCurrentState)
                     argos::LOG << "thread state: " << threadState << '\n';

                  argos::LOG << "prepare to push done: " << prepareToPushDone << '\n';

                  /* Find absolute distance between point and robot and assign shortest distance to each robot */               
                  for(size_t i = 0; i < validPushPoints.size(); i++)
                  {
                     int idxPH = cornerAllocation(startLocations, validPushPoints[i], isRobotAssigned);
                     isRobotAssigned[idxPH] = true;
                     argos::LOG << "indexPH: " << idxPH << std::endl;

                     /* Get image of the current map & draw endpoints */
                     cameraImage = cam.GetPlot();
                     cv::circle(cameraImage, convertToCV( boxGoal ), 7, cv::Scalar(0,200,255), -1 );
                     drawEndPoints(i, plan, pcBox, validPushPoints, boxGoal, cameraImage);

                     /* Define a kernel and erode the map in order to not get close to obstacles */
                     int dilation_size = 0.15*SCALE;
                     cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                        cv::Point( dilation_size, dilation_size ) );
                     cv::erode(cameraImage, cameraImage, kernel);

                     /* Planning */
                     std::vector<cv::Point> subGoals;
                     bool planComplete = Planning(cameraImage, boxGoal, startLocations[idxPH], validPushPoints[i], subGoals);

                     /* Visualization */
                     // for(auto point : subGoals)
                     //    cv::circle(cameraImage, point, 5, cv::Scalar(0,200,200), -1);
                     // cv::circle(cameraImage, cv::Point(startLocations[idxPH].GetX()*SCALE, startLocations[idxPH].GetY()*SCALE) , 5, cv::Scalar(0,0,255), -1);
                     // cv::circle(cameraImage, subGoals.back(), 5, cv::Scalar(0,255,0), -1);
                     // cv::imshow("map", cameraImage);
                     // cv::waitKey(0);

                     /* Start thread */
                     argos::LOG << "subGoals: " << subGoals.size() << std::endl;
                     robotThreads[idxPH] = std::thread(&cameraServerLoop::PrepareToPush, this, boxGoal, 
                                                            subGoals, PLANNING, idxPH);
                     robotThreads[idxPH].detach();

                  }

                  threadsOpened = true;
               }
               else
               {
                  /*Debug*/
                  //debugFun(clientcount, false);

                  /* Checks for the robots' states and moves on if all are in WAIT state */
                  bool stateCheck = true;
                  for(size_t i = 0; i < threadCurrentState.size(); i++)
                  {
                     stateCheck &= threadCurrentState[i] == WAIT;
                  }
 
                  if(stateCheck)
                     currentState = JOIN_THREADS;

               }
               break;
            }

            /************************* JOIN_THREADS *************************/
            case JOIN_THREADS:
            {
               argos::LOG << "SERVER JOIN_THREADS\n";

               /* This will end the while loop running in the thread making them exit */
               prepareToPushDone = true;
               bool closed = true;
               for (size_t i = 0; i < threadClosed.size(); i++)
               {
                  closed &= threadClosed[i];
               }
               if (closed)
               {
                  currentState = SEND_VELOCITY;
               }
               
               break;
            }

            /************************* SEND_VELOCITY *************************/
            case SEND_VELOCITY:
            {
               argos::LOG << "SERVER SEND_VELOCITY\n";
               argos::Real velocityMessage = 2.0f;
               bool velReceived = true;
               for(int i = 0; i < clientcount; i++)
               {
                  velReceived &= clientConnections[i].send(velocityMessage);
               }
               if(velReceived);
               {
                  currentState = WAIT;
               }
               break;
            }

            /************************* WAIT *************************/
            case WAIT:
            {
               argos::LOG << "SERVER WAIT\n";

               bool inRange = serverWaitState(pcBox, boxGoal, clientcount, clientConnections, GOAL_THRESHOLD, true);
               if(inRange)
                  currentState = SEND_STOP;

               stopSent_ = true;
               break;
            }

            /************************* SEND_STOP *************************/
            case SEND_STOP:
            {
               argos::LOG << "SERVER SEND_STOP\n";
               rewind = true;
               argos::Real velocityMessage = -2.0f;
               argos::Real velocityStopMessage = 0.0f;

               if (rewind_ && footbotStopped_ && stopSent_)
               {
                  currentState = DONE;
               }

               //send STOP message
               if (!stopSent_)
               {
                  cv::waitKey(10);
                  stopSent = true;
                  for(int i = 0; i < clientcount && !stopSent_; i++)
                  {
                     stopSent &= clientConnections[i].send("STOP");
                     argos::LOG << "send STOP message\n";
                  }
                  stopSent_ = stopSent;// true if all bots have recieved 
               }

               //set velocity to 0
               if (!footbotStopped_ && rewind_ && stopSent_)
               {
                  if (time(0)-backTime > 2)
                  {
                     cv::waitKey(10);
                     footbotStopped = true;
                     for(int i = 0; i < clientcount && !footbotStopped_; i++) 
                     {
                        footbotStopped &= clientConnections[i].send(velocityStopMessage);
                        argos::LOG << "---- send footbot Stopped message\n";
                     }
                     footbotStopped_ = footbotStopped; // true if all bots have recieved
                     stopSent_ = !footbotStopped_; //set stopSet to false to it sents stop again
                  }
               }

               //back away from box
               if (!rewind_  && stopSent_)
               {
                  cv::waitKey(10);
                  for(int i = 0; i < clientcount; i++)
                  {
                     rewind &= clientConnections[i].send(velocityMessage);
                     backTime = time(0);
                     argos::LOG << "send rewind message\n";
                  }
                  rewind_ = rewind;// true if all bots have recieved 
                  stopSent_ = !rewind_; //set stopSet to false to it sents stop again

               }
               
               

               break;
            }
            case DONE:
            {
               std::cout << "SERVER DONE\n";
               jobsDone = true;

            }
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
   connected = true;
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
   bool threadDone = false;
   int curGoal = 0;
   int time;
   argos::CVector3 robotPosition;
   while(!threadDone)
   {
      // if (threadDone)
      //    break;      
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
               threadDone = true;
               threadCurrentState[id] = WAIT;
            }
         }
         break;
      }
      default:
      break;
      }
   }
   threadClosed[id] = true;
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
int cameraServerLoop::portnumber = 0;