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
    argos::CFootBotEntity* fBot;

    /* socket stuff*/
    argos::CTCPSocket serverSocket;
    std::vector<argos::CTCPSocket> clientSockets;
    std::vector<argos::CVector3> robotPositions;
    std::vector<protocol> clientConnections;
    int clientConnected;
    bool connected;
    void connect_();

    /* camera stuff */
    camera C;
    camera cam;
    cv::Mat cameraImage;

    /* debug stuff */
    std::vector<double> debug;
    std::vector<cv::Mat> debugMaps;

    /* uncategorized stuff */
    argos::CVector3 boxGoal;                            //final destination for the box
    std::vector<argos::CVector3> startLocations;        //robot location
    int currentState;                                   //cameraServerloop state
    std::vector<int> threadCurrentState;                //to check if thread is in wait state
    int stateCheck;                                     //to check if all thread is in wait state
    bool threadsOpened;                                 //used to open threads 1 time only
    std::vector<bool> recievedPosition;                 //to check if bots position has been recieved
    bool allPositionRecieved;                           //to check if all bots position has been recieved
    bool prepareToPushDone;                             //to stop all thread from running
    planner plan;                                       //plan



public:
    static int portnumber;

    void operator()(int clientcount_, argos::CVector3 boxGoal_, argos::CBoxEntity* pcBox_);
    cameraServerLoop();
    ~cameraServerLoop();
    
    void connect();
    void step();


    bool Planning(cv::Mat &map, argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, std::vector<cv::Point> &subGoals);//, int id);

    // void PrepareToPush(cv::Mat map, argos::CVector3 goal, argos::CVector3 startLoc, 
    //                                    argos::CVector3 cornerLoc, int currentState_, int id);
    void PrepareToPush(argos::CVector3 boxGoal, std::vector<cv::Point> subGoals, int currentState_, int id);

};








#endif // __CAMERASERVERLOOPBACK_H__