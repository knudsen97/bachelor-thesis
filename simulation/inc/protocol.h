#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <thread>

#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector3.h>
#include <opencv2/core.hpp>



class protocol
{
public:
    enum dataType
    {
        typeString,
        typeCVector3,
        typeCRadians,
        typeReal,
        typeCByteArray,
        typePoint
    };
    
    protocol(protocol &&) = default;
    protocol(argos::CTCPSocket& socket);
    protocol();
    ~protocol();
    void operator()(argos::CTCPSocket& socket);

    /****************************************
    ***** transmit part of the protocol *****
    *****************************************/

    /** @name send
     * @brief Sends a datapackage
     * @param message The data to send.
     * @retval True if it has been sent false otherwise.
     */
    bool send(argos::CByteArray message);

    /** @name send
     * @brief Sends a datapackage
     * @param message The data to send.
     * @retval True if it has been sent false otherwise.
     */
    bool send(std::string message);

    /** @name send
     * @brief Sends a datapackage
     * @param message The data to send.
     * @retval True if it has been sent false otherwise.
     */
    bool send(argos::CVector3 message);

    /** @name send
     * @brief sends a datapackage
     * @param X The radian to send.
     * @param Y The radian to send.
     * @param Z The radian to send.
     * @retval True if it has been sent false otherwise.
     */
    bool send(argos::CRadians X, argos::CRadians Y, argos::CRadians Z);
    
    /** @name send
     * @brief Sends a datapackage
     * @param message The data to send.
     * @retval True if it has been sent false otherwise.
     */
    bool send(argos::Real message); 
    
    /** @name send
     * @brief Sends a datapackage
     * @param message The data to send.
     * @retval True if it has been sent false otherwise.
     */
    bool send(cv::Point message);


    /****************************************
    ****** recieve part of the protocol *****
    *****************************************/


    /** @name sendPosition
     * @brief sends a position (legacy code please use @ref send)
     * @param position The position to send.
     * @retval true if it has been sent false otherwise.
     */
    bool sendPosition(argos::CVector3 position);



    /** @name recieve
     * @brief recieves a message.
     * @param message The buffer in which the message is saved.
     * @retval true if it did recieve anyting else false.
     */
    bool recieve(argos::CByteArray& message);

    /** @name recieve
     * @brief recieves a message.
     * @param message The buffer in which the message is saved.
     * @retval true if it did recieve anyting else false.
     */
    bool recieve(std::string& message);

    
    
    /** @name recieve
     * @brief recieves a message.
     * @param message The buffer in which the message is saved.
     * @retval true if it did recieve anyting else false.
     */
    bool recieve(argos::CVector3& message);
    
    /** @name recieve
     * @brief recieves a message.
     * @param message The buffer in which the message is saved.
     * @retval true if it did recieve anyting else false.
     */
    bool recieve(argos::Real& message);
    
    /** @name recieve
     * @brief recieves a message.
     * @param message The buffer in which the message is saved.
     * @retval true if it did recieve anyting else false.
     */
    bool recieve(cv::Point& message);
    
    /** @name recieve
     * @brief recieves anyting without saving it
     */
    bool recieve();

    
    /** @name recieve
     * @brief recieves a message.
     * @param X The buffer for X.
     * @param Y The buffer for Y.
     * @param Z The buffer for Z.
     * @retval true if it did recieve anyting else false.
     */
    bool recieve(argos::CRadians& X, argos::CRadians& Y, argos::CRadians& Z);


    
    bool recievePosition(argos::CVector3& message);

private:
    argos::CTCPSocket* socket;
    bool sendt = true;
    bool recieved = true;
    void sendMessage(argos::CByteArray message, dataType type);

    void recieveMessage();

    std::thread communicate;
    argos::CByteArray recievedMessage, recievedMessageLength;
    argos::UInt8 messageLength[2];
    argos::UInt8 preMessageRecieved[2];

};


#endif // __PROTOCOL_H__