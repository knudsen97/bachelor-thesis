#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <thread>

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/utility/logging/argos_log.h>



class protocol
{
public:
    protocol(protocol &&) = default;
    protocol(argos::CTCPSocket& socket);
    protocol();
    ~protocol();
    bool send(argos::CByteArray message);
    bool send(std::string message);
    bool messageHasBeenSendt();
    bool recieve(argos::CByteArray& message);
    void operator()(argos::CTCPSocket& socket);

private:
    /* data */
    argos::CTCPSocket* socket;
    bool sendt = true;
    bool recieved = true;
    void sendMessage(argos::CByteArray message);
    void recieveMessage();
    void setUpServer();
    void setUpClient();
    std::thread communicate;
    argos::CByteArray recievedMessage, recievedMessageLength;
    argos::UInt8 messageLength[1];
};


#endif // __PROTOCOL_H__