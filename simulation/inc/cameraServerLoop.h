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
#include "../inc/protocol.h"
#include "../inc/camera.h"


using namespace argos;

class cameraServerLoop
{
protected:
    /* data */
    static int clientcount;
    static int portnumber;

    static CBoxEntity* pcBox;
    static CFootBotEntity* fBot;

    /* socket stuff*/
    argos::CTCPSocket serverSocket;
    std::vector<argos::CTCPSocket> clientSockets;
    std::vector<argos::CVector3> robotPositions;
    std::vector<protocol> clientConnections;
    int clientConnected;


    bool positionRecieved;

    std::vector<CVector3> startLocations;
    camera C;

    bool cornerFound;

    int currentState;

    std::vector<int> threadCurrentState;

    bool threadsOpened;

    std::vector<bool> recievedPosition;

    bool allPositionRecieved;

    bool prepareToPushDone;
    int stateCheck;

    void connect_();

    camera cam;
    cv::Mat cameraImage;

    planner plan;


    std::vector<double> debug;
    std::vector<cv::Mat> debugMaps;


public:
    void operator()(int clientcount_);
    cameraServerLoop();
    
    void connect();
    void step();


    bool Planning(cv::Mat &map, argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, std::vector<cv::Point> &subGoals);//, int id);

    // void PrepareToPush(cv::Mat map, argos::CVector3 goal, argos::CVector3 startLoc, 
    //                                    argos::CVector3 cornerLoc, int currentState_, int id);
    void PrepareToPush(argos::CVector3 boxGoal, std::vector<cv::Point> subGoals, int currentState_, int id);

};








#endif // __CAMERASERVERLOOPBACK_H__