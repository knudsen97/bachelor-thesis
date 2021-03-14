#include "../inc/cameraServerLoop.h"


void cameraServerLoop::init() 
{
   std::thread connecting(connect);
   connecting.join();
}

void cameraServerLoop::step()
{
   for (size_t i = 0; i < clientcount; i++)
   {
      clientConnections[i].send("hej\n");
   }
}

void cameraServerLoop::connect()
{
   serverSocket.Listen(portnumber);
   clientSockets.resize(clientcount);
   for (size_t i = 0; i < clientcount; i++)
   {
      serverSocket.Accept(clientSockets[i]);
      clientConnections.push_back(protocol(clientSockets[i]));
   }
}

int cameraServerLoop::clientcount = 0;
int cameraServerLoop::portnumber = 0;
argos::CTCPSocket cameraServerLoop::serverSocket;
std::vector<argos::CTCPSocket> cameraServerLoop::clientSockets;
std::vector<protocol> cameraServerLoop::clientConnections;
