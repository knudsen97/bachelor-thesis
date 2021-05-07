#ifndef __CAMERASERVERLOOPBACK_H__
#define __CAMERASERVERLOOPBACK_H__

#include <vector>
#include <iostream>
#include <thread>
#include <ctime>

#include <argos3/core/utility/configuration/argos_configuration.h>

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "cameraServerLoop.h"
#include "test_controller.h"
#include "protocol.h"
#include "camera.h"


class cameraServerLoop {
protected:
    /* data */
    int clientcount;

    argos::CBoxEntity* pcBox;
    argos::CPrototypeEntity* pcObject;
    argos::CEPuckEntity* fBot;
    std::string servername;

    /* socket stuff*/
protected:
    static argos::CTCPSocket serverSocket;
    static std::vector<argos::CTCPSocket> clients;
    static std::vector<argos::CTCPSocket*> clientSockets;
    std::vector<argos::CVector3> robotPositions;
    std::vector<protocol> clientConnections;
    int clientConnected;
    bool connected;
    static void connect_(int totalClientCount);
    static std::vector<std::pair<std::string, argos::CTCPSocket*>> robotSocketPair;

public:
    void connect();
    static void establishConnection(int totalClientCount);
    static int portnumber;
    static bool everyoneIsConected;

    /* camera stuff */
protected:
    camera C;
    camera cam;
    cv::Mat cameraImage;

    /* debug stuff */
    std::vector<bool> debug;
    std::vector<cv::Mat> debugMaps;

    /* uncategorized stuff */
    argos::CVector3 boxGoal;                            //final destination for the box
    std::vector<argos::CVector3> startLocations;        //robot location
    std::vector<int> threadCurrentState;                //to check if thread is in wait state
    planner plan;                                       //plan
    

    /* flags */
    bool allPositionRecieved;                           //to check if all bots position has been recieved
    bool prepareToPushDone;                             //to stop all thread from running
    bool threadsOpened;                                 //used to open threads 1 time only
    std::vector<bool> recievedPosition;                 //to check if bots position has been recieved
    std::vector<bool> threadClosed;

public:    
    bool rewind, rewind_;                               //set if the footbot needs to go backwards
    bool stopSent, stopSent_;                           //set if "STOP" has been sent to the footbot
    bool footbotStopped, footbotStopped_;                 //set if the velocity sent to footbot is 0
    bool inRange_;                                      //set if box is in range
    bool jobsDone;                                      //set if servers jobe is done
    int currentState;                                   //cameraServerloop state
    size_t backTime;                                    //time to back away from box
    size_t argosTime;                                   //time unit in argos
    int objectType;

public:
    void operator()(argos::CVector3 boxGoal_, argos::CBoxEntity* pcBox_);
    void operator()(int clientcount_, argos::CVector3 boxGoal_, argos::CBoxEntity* pcBox_, std::string servername);
    
    void operator()(argos::CVector3 boxGoal_, argos::CPrototypeEntity* pcBox_);
    void operator()(int clientcount_, argos::CVector3 boxGoal_, argos::CPrototypeEntity* pcBox_, std::string servername);

    cameraServerLoop();
    ~cameraServerLoop();
    
    void step();


    bool Planning(cv::Mat &map, argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, std::vector<cv::Point> &subGoals);//, int id);

    // void PrepareToPush(cv::Mat map, argos::CVector3 goal, argos::CVector3 startLoc, 
    //                                    argos::CVector3 cornerLoc, int currentState_, int id);
    void PrepareToPush(argos::CVector3 boxGoal, std::vector<cv::Point> subGoals, int currentState_, int id);

};








#endif // __CAMERASERVERLOOPBACK_H__