#include "../inc/protocol.h"

protocol::protocol(argos::CTCPSocket& socket_)
{
    socket = &socket_;
    // sendt = socket->GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) != socket->GetEvents().end();
    // recieved = socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) == socket->GetEvents().end();
}
void protocol::operator()(argos::CTCPSocket& socket_)
{
    socket = &socket_;
    // sendt = socket->GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) != socket->GetEvents().end();
    // recieved = socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) == socket->GetEvents().end();
}

protocol::protocol(){}

protocol::~protocol(){}

bool protocol::send(argos::CByteArray message) 
{
    if (sendt && recieved)
    {
        communicate = std::thread(&protocol::sendMessage, this, message);
        communicate.detach();
        sendt = false;
        return true;
    }
    else
        return false;
}
bool protocol::send(std::string message_) 
{
    argos::CByteArray message = argos::CByteArray((argos::UInt8*)message_.c_str(), message_.size()+1);
    return protocol::send(message);
}

void protocol::sendMessage(argos::CByteArray message)
{
    argos::UInt8 premessage[2];
    premessage[0] = (message.Size() + '0');
    while(socket->GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) == socket->GetEvents().end())
    {
        ;
    }
    socket->SendBuffer(premessage, 1);

    while(socket->GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) == socket->GetEvents().end())
    {
        ;
    }
    socket->SendByteArray(message);
    sendt = true;
}


bool protocol::messageHasBeenSendt() 
{
    return sendt;
}


bool protocol::recieve(argos::CByteArray& message) 
{
    std::cout << "sendt: " << sendt << '\n';
    std::cout << "recieved: " << recieved << '\n';
    if (recieved && sendt)
    {
        std::cout << "new recieve thread\n";
        communicate = std::thread(&protocol::recieveMessage, this);
        communicate.detach();
        recieved = false;
    }
    else
    {
        std::cout << "could not listen\n";
        return false;
    }
    usleep(500);
    if (recieved)
    {
        std::cout << "recieved\n";
        message = recievedMessage;
        return true;
    }
    else
    {
        return false;
    }
    
    
}

void protocol::recieveMessage() 
{
    while (socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) == socket->GetEvents().end())
    {
        ;
    }
        socket->ReceiveBuffer(messageLength, 1);

    while (socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) == socket->GetEvents().end())
    {
        ;
    }
        argos::CByteArray temp(messageLength[0]);
        socket->ReceiveByteArray(temp);
        recievedMessage = temp;
    

    recieved = true;
}