#ifndef __CAMERASERVERLOOPBACK_H__
#define __CAMERASERVERLOOPBACK_H__

#include <vector>
#include <iostream>
#include <thread>

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "cameraServerLoop.h"
#include "test_controller.h"





class cameraServerLoop
{
protected:
    /* data */
    static int clientcount;
    static int portnumber;
    static char buffer[5]; 
    static argos::CTCPSocket serverSocket;
    static std::vector<argos::CTCPSocket> clientSockets;

public:
    static void init();
    static void step();
    static void connect();


};








#endif // __CAMERASERVERLOOPBACK_H__