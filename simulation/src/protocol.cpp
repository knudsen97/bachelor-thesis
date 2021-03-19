#include "../inc/protocol.h"

protocol::protocol(argos::CTCPSocket& socket_)
{
    socket = &socket_;
    preMessageRecieved[0] = 0;
    preMessageRecieved[1] = typeError;
}
void protocol::operator()(argos::CTCPSocket& socket_)
{
    socket = &socket_;
    preMessageRecieved[0] = 0;
    preMessageRecieved[1] = typeError;
}

protocol::protocol(){}

protocol::~protocol(){}

/****************************************
***** transmit part of the protocol *****
*****************************************/
/* public send code they all spawn a thread */

/** @name send
 * @brief Sends a datapackage
 * @param message The data to send.
 * @retval True if it has been sent false otherwise.
 */
bool protocol::send(argos::CByteArray message) 
{
    if (sendt && recieved)
    {
        communicate = std::thread(&protocol::sendMessage, this, message, dataType::typeCByteArray);
        communicate.detach();
        sendt = false;
        return true;
    }
    else
        return false;
}
/** @name send
 * @brief Sends a datapackage
 * @param message The data to send.
 * @retval True if it has been sent false otherwise.
 */
bool protocol::send(std::string message) 
{
    if (sendt && recieved)
    {
        argos::CByteArray package;
        package = argos::CByteArray((argos::UInt8*)message.c_str(), message.size()+1);
        communicate = std::thread(&protocol::sendMessage, this, package, dataType::typeString);
        communicate.detach();
        sendt = false;
        return true;
    }
    else
        return false;
}
/** @name send
 * @brief Sends a datapackage
 * @param message The data to send.
 * @retval True if it has been sent false otherwise.
 */
bool protocol::send(argos::CVector3 message) 
{
    if (sendt && recieved)
    {        
        std::string strMessage;
        strMessage.append(std::to_string(message.GetX()) + ' ');
        strMessage.append(std::to_string(message.GetY()) + ' ');
        strMessage.append(std::to_string(message.GetZ()));

        argos::CByteArray package;
        package = argos::CByteArray((argos::UInt8*)strMessage.c_str(), strMessage.size()+1);
        communicate = std::thread(&protocol::sendMessage, this, package, dataType::typeCVector3);
        communicate.detach();
        sendt = false;
        return true;
    }
    else
        return false;

}
/** @name send
 * @brief sends a datapackage
 * @param X The radian to send.
 * @param Y The radian to send.
 * @param Z The radian to send.
 * @retval True if it has been sent false otherwise.
 */
bool protocol::send(argos::CRadians X, argos::CRadians Y, argos::CRadians Z) 
{
    if (sendt && recieved)
    {        
        std::string strMessage;
        strMessage.append(std::to_string(X.GetValue()) + ' ');
        strMessage.append(std::to_string(Y.GetValue()) + ' ');
        strMessage.append(std::to_string(Z.GetValue()));

        argos::CByteArray package;
        package = argos::CByteArray((argos::UInt8*)strMessage.c_str(), strMessage.size()+1);
        communicate = std::thread(&protocol::sendMessage, this, package, dataType::typeCRadians);
        communicate.detach();
        sendt = false;
        return true;
    }
    else
        return false;
}
/** @name send
 * @brief Sends a datapackage
 * @param message The data to send.
 * @retval True if it has been sent false otherwise.
 */
bool protocol::send(argos::Real message) 
{
    if (sendt && recieved)
    {   
        u_char* prePackage = reinterpret_cast<u_char*>(&message);
        argos::CByteArray package(prePackage, sizeof(prePackage)/sizeof(u_char));
        communicate = std::thread(&protocol::sendMessage, this, package, dataType::typeReal);
        communicate.detach();
        sendt = false;
        return true;
    }
    else
        return false;
}
/** @name send
 * @brief Sends a datapackage
 * @param message The data to send.
 * @retval True if it has been sent false otherwise.
 */
bool protocol::send(cv::Point message) 
{
    if (sendt && recieved)
    {   
        std::string strMessage;
        strMessage.append(std::to_string(message.x) + ' ');
        strMessage.append(std::to_string(message.y));

        argos::CByteArray package;
        package = argos::CByteArray((argos::UInt8*)strMessage.c_str(), strMessage.size()+1);
        communicate = std::thread(&protocol::sendMessage, this, package, dataType::typePoint);
        communicate.detach();
        sendt = false;

        return true;
    }
    else
        return false;
}


/** @name sendPosition
 * @brief sends a position (legacy code please use @ref send(argos::CVector3) )
 * @param position The position to send.
 * @retval true if it has been sent false otherwise.
 */
bool protocol::sendPosition(argos::CVector3 position) 
{
    std::string message;
    message.append(std::to_string(position.GetX()) + ' ');
    message.append(std::to_string(position.GetY()) + ' ');
    message.append(std::to_string(position.GetZ()));
    return send(message);
}


/** the private thread code **/
void protocol::sendMessage(argos::CByteArray message, dataType type) 
{
    argos::UInt8 preMessage[2];
    preMessage[0] = message.Size();
    preMessage[1] = type;
    while(socket->GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) == socket->GetEvents().end())
    {
        ;
    }
    socket->SendBuffer(preMessage, 2);

    while(socket->GetEvents().find(argos::CTCPSocket::EEvent::OutputReady) == socket->GetEvents().end())
    {
        ;
    }
    socket->SendByteArray(message);
    sendt = true;
}



/****************************************
****** recieve part of the protocol *****
*****************************************/

/** @name recieve
 * @brief Recieves a message.
 * @param message The buffer in which the message is saved.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recieve(argos::CByteArray& message) 
{
    // std::cout << "sendt: " << sendt << '\n';
    // std::cout << "recieved: " << recieved << '\n';
    if (recieved && sendt && socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) != socket->GetEvents().end())
    {
        
        communicate = std::thread(&protocol::recieveMessage, this);
        communicate.detach();
        recieved = false;
    }
    else
        return false;
    usleep(500);
    if (recievedMessage.Size()>0)
    {
        message = recievedMessage;
        communicate.~thread();
        recieved = true;
        return true;
    }
    else
        return false;
}
/** @name recieve
 * @brief Recieves a message.
 * @param message The buffer in which the message is saved.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recieve(std::string& message) 
{
    argos::CByteArray package;
    bool packageRecieved = recieve(package);
    if(packageRecieved && preMessageRecieved[1] == dataType::typeString)
    {
        package >> message;
        return true;
    }
    return false;
}
/** @name recieve
 * @brief Recieves a message.
 * @param message The buffer in which the message is saved.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recieve(argos::CVector3& message) 
{
    argos::CByteArray package;
    bool packageRecieved = recieve(package);
    if(packageRecieved && preMessageRecieved[1] == dataType::typeCVector3)
    {
        std::string str_message;
        package >> str_message;

        argos::Real X = std::stod(str_message.substr(0, str_message.find(' ')));
        str_message.erase(0, str_message.find(' ') + 1);

        argos::Real Y = std::stod(str_message.substr(0, str_message.find(' ')));
        str_message.erase(0, str_message.find(' ') + 1);

        argos::Real Z = std::stod(str_message.substr(0, str_message.find(' ')));
        
        message.SetX(X);
        message.SetY(Y);
        message.SetZ(Z);
        return true;
    }

    //should not get here if everything went well
    return false;
}
/** @name recieve
 * @brief Recieves a message.
 * @param message The buffer in which the message is saved.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recieve(argos::Real& message) 
{
    argos::CByteArray package;
    bool packageRecieved = recieve(package);
    if(packageRecieved && preMessageRecieved[1] == dataType::typeReal)
    {
        double preMessage = *reinterpret_cast<double*>(package.ToCArray());
        message = preMessage;
        return true;
    }
    return false;
}
/** @name recieve
 * @brief Recieves a message.
 * @param message The buffer in which the message is saved.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recieve(cv::Point& message) 
{
    argos::CByteArray package;
    bool packageRecieved = recieve(package);
    if(packageRecieved && preMessageRecieved[1] == dataType::typePoint)
    {
        std::string str_message;
        package >> str_message;

        int X = std::stod(str_message.substr(0, str_message.find(' ')));
        str_message.erase(0, str_message.find(' ') + 1);

        int Y = std::stod(str_message.substr(0, str_message.find(' ')));

        message.x = X;
        message.y = Y;
        return true;
    }
    return false;
}
/** @name recieve
 * @brief recieves anyting without saving it
 */
bool protocol::recieve()
{
    argos::CByteArray nothing;
    return recieve(nothing);
} 

/** @name recieve
 * @brief Recieves a message.
 * @param X The buffer for X.
 * @param Y The buffer for Y.
 * @param Z The buffer for Z.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recieve(argos::CRadians& X_, argos::CRadians& Y_, argos::CRadians& Z_) 
{
    argos::CByteArray package;
    bool packageRecieved = recieve(package);
    if(packageRecieved && preMessageRecieved[1] == dataType::typeCRadians)
    {
        std::string str_message;
        package >> str_message;

        argos::Real X = std::stod(str_message.substr(0, str_message.find(' ')));
        str_message.erase(0, str_message.find(' ') + 1);

        argos::Real Y = std::stod(str_message.substr(0, str_message.find(' ')));
        str_message.erase(0, str_message.find(' ') + 1);

        argos::Real Z = std::stod(str_message.substr(0, str_message.find(' ')));
        
        X_.SetValue(X);
        Y_.SetValue(Y);
        Z_.SetValue(Z);
        return true;
    }

    //should not get here if everything went well
    return false;
}


/** @name recievePosition
 * @brief Recieves a argos::CVector3 (legacy code please use @ref recieve).
 * @param position The buffer in which the position is saved.
 * @retval True if it did recieve anyting else false.
 */
bool protocol::recievePosition(argos::CVector3& position) 
{
    argos::CByteArray message;
    std::string str_message;
    
    if(!this->recieve(message))
        return false;
    
    message >> str_message;

    argos::Real X = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    argos::Real Y = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    argos::Real Z = std::stod(str_message.substr(0, str_message.find(' ')));
    
    position.SetX(X);
    position.SetY(Y);
    position.SetZ(Z);
    
    return true;
}


/*********************************************************
** functions to peak a message rather then recieving it **
*********************************************************/

protocol::dataType protocol::getMessageType() 
{    
    return protocol::dataType(preMessageRecieved[1]);
}

argos::CByteArray protocol::getMessage(argos::CByteArray& message) 
{
    message = recievedMessage;
    return recievedMessage;
}

std::string protocol::getMessage(std::string& message) 
{
    std::string temp;
    recievedMessage >> temp;
    message = temp;
    return temp;
}

argos::CVector3 protocol::getMessage(argos::CVector3& message) 
{
    std::string str_message;
    argos::CVector3 temp;
    recievedMessage >> str_message;

    argos::Real X = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    argos::Real Y = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    argos::Real Z = std::stod(str_message.substr(0, str_message.find(' ')));
    
    temp.SetX(X);
    temp.SetY(Y);
    temp.SetZ(Z);

    message = temp;
    return temp;
}

argos::Real protocol::getMessage(argos::Real& message) 
{
    double temp = *reinterpret_cast<double*>(recievedMessage.ToCArray());
    message = temp;
    return temp;
}

cv::Point protocol::getMessage(cv::Point& message) 
{
    std::string str_message;
    cv::Point temp;
    recievedMessage >> str_message;

    int X = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    int Y = std::stod(str_message.substr(0, str_message.find(' ')));

    temp.x = X;
    temp.y = Y;
    message = temp;
    return temp;
}

std::vector<argos::CRadians> protocol::getMessage(argos::CRadians& X_, argos::CRadians& Y_, argos::CRadians& Z_) 
{

    std::string str_message;
    std::vector<argos::CRadians> temp;
    temp.resize(3);
    recievedMessage >> str_message;

    argos::Real X = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    argos::Real Y = std::stod(str_message.substr(0, str_message.find(' ')));
    str_message.erase(0, str_message.find(' ') + 1);

    argos::Real Z = std::stod(str_message.substr(0, str_message.find(' ')));
    
    temp[0].SetValue(X);
    temp[1].SetValue(Y);
    temp[2].SetValue(Z);

    X_ = temp[0];
    Y_ = temp[1];
    Z_ = temp[2];

    return temp;
}



/** the private thread code **/
void protocol::recieveMessage() 
{
    while (socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) == socket->GetEvents().end());
    socket->ReceiveBuffer(preMessageRecieved, 2);
    recievedMessage.Clear();


    while (socket->GetEvents().find(argos::CTCPSocket::EEvent::InputReady) == socket->GetEvents().end());
    argos::CByteArray temp(preMessageRecieved[0]);

    socket->ReceiveByteArray(temp);
    recievedMessage = temp;
    recieved = true;
}


    argos::CByteArray protocol::dummy_CByte = {0};
    std::string protocol::dummy_string = "";
    argos::CVector3 protocol::dummy_CVector3 = {0,0,0};
    argos::CRadians protocol::dummy_CRadians(0);
    argos::Real protocol::dummy_Real = 0;
    cv::Point protocol::dummy_Point = {0, 0};
