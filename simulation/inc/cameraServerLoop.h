#ifndef __CAMERASERVERLOOPBACK_H__
#define __CAMERASERVERLOOPBACK_H__

#include <vector>
#include <iostream>
#include <thread>


#include <argos3/core/utility/configuration/argos_configuration.h>

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "cameraServerLoop.h"
#include "test_controller.h"
#include "../inc/protocol.h"


using namespace argos;

class cameraServerLoop
{
protected:
    /* data */
    static int clientcount;
    static int portnumber;
    static argos::CTCPSocket serverSocket;
    static std::vector<argos::CTCPSocket> clientSockets;
    static std::vector<argos::CVector3> robotPositions;
    static std::vector<protocol> clientConnections;
    static bool positionRecieved;

    static CBoxEntity* pcBox;
    static CFootBotEntity* fBot;

    static std::vector<CVector3> startLocations;
    static camera C;

    static bool cornerFound;

    static int currentState;

    static std::vector<int> threadCurrentState;

    static bool threadsOpened;

    static std::vector<bool> recievedPosition;

    static bool allPositionRecieved;

    static bool prepareToPushDone;
    static int stateCheck;

public:
    static void init();
    static void step();
    static void connect();

    static bool Planning(argos::CVector3 goal, argos::CVector3 startLoc, argos::CVector3 cornerLoc, std::vector<cv::Point> &subGoals, int id);

    static void PrepareToPush(argos::CVector3 goal, argos::CVector3 startLoc, 
                                argos::CVector3 cornerLoc, int currentState, int id);
};








#endif // __CAMERASERVERLOOPBACK_H__