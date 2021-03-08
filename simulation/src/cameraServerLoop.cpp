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
      std::string temp;
      if(clientSockets[i].GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) != clientSockets[i].GetEvents().end())
      {
         for (size_t j = 0; j < test_controller::robotBufferSize-std::to_string(i).length() -2; j++)
         {
            temp.append("0");
         }
         temp.append(std::to_string(i+1));
         temp.append("\n");
         argos::CByteArray message((argos::UInt8*)temp.c_str(), temp.size()+1);
         clientSockets[i].SendByteArray(message);
         argos::LOG << "message sent was: " << message << '\n';
      }
   }
}

void cameraServerLoop::connect()
{
   serverSocket.Listen(portnumber);
   clientSockets.resize(clientcount);
   for (size_t i = 0; i < clientcount; i++)
   {
      serverSocket.Accept(clientSockets[i]);
   }
}

int cameraServerLoop::clientcount = 0;
int cameraServerLoop::portnumber = 0;
argos::CTCPSocket cameraServerLoop::serverSocket;
std::vector<argos::CTCPSocket> cameraServerLoop::clientSockets;
char cameraServerLoop::buffer[5] = {0};
