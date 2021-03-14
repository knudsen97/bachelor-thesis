#include "../inc/cameraServerLoop.h"


void cameraServerLoop::init() 
{
   std::thread connecting(connect);
   connecting.join();
}

void cameraServerLoop::step()
{
   if (!positionRecieved)
   {
      robotPositions.resize(clientcount);
      for (size_t i = 0; i < clientcount; i++)
      {
         clientConnections[i].recievePosition(robotPositions[i]);
      }
      positionRecieved = true;
   }
   
   argos::LOG << "rob pos: " << robotPositions[0] << '\n';
   
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
std::vector<argos::CVector3> cameraServerLoop::robotPositions;
std::vector<protocol> cameraServerLoop::clientConnections;
bool cameraServerLoop::positionRecieved = false;
