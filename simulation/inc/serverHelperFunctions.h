
/**
 * @brief This function converts a ArgOS CVector3 to an OpenCV Point.
 * @param arg An ArgOS CVector3
**/
cv::Point convertToCV(argos::CVector3 arg)
{
   return cv::Point(arg.GetX()*SCALE, arg.GetY()*SCALE);
}

/**
 * @brief Find which robot is closest to which corner 
 * @param _startLocation The robots' start locations
 * @param _validPushPoint One of the corners of the box
 * @param _isRobotAssigned To check if the robot already has been assigned a corner
*/
int cornerAllocation(std::vector<argos::CVector3> _startLocations, argos::CVector3 _validPushPoint, std::vector<bool> _isRobotAssigned)
{
    double shortestDistance = 999999.99f, PH = 0.0f;
    int idxPH = 0;
    for(size_t j = 0; j < _startLocations.size(); j++)
    {
        PH = abs( _startLocations[j].GetX() - _validPushPoint.GetX() 
                + _startLocations[j].GetY() - _validPushPoint.GetY());

        if(PH < shortestDistance && _isRobotAssigned[j] == false)
        {
            shortestDistance = PH;
            idxPH = j;
        }
    }

    return idxPH;
}


/**
 * @brief Draws the other robots endpoint on the map in order to avoid collisions/robots getting stuck by not being able to reach their point.
 * @param _i The current iteration of which corner is being inspected
 * @param _plan An instance of the planner class
 * @param _pcBox The argos box entity
 * @param _validPushPoints The push points on the _pcBox
 * @param _boxGoal The goal location of the box
 * @param _cameraImage A cv::Mat of the current map
*/
void drawEndPoints(int _i, planner &_plan, argos::CBoxEntity* _pcBox, 
                    std::vector<argos::CVector3> _validPushPoints, argos::CVector3 _boxGoal, cv::Mat &_cameraImage)
{
    argos::CVector3 robotEndPoint(0,0,0);

    for (size_t k = 0; k < _validPushPoints.size(); k++)
    {
        if (k != _i)
        {
            // robotEndPoint = _plan.push(_pcBox, _validPushPoints[k], _boxGoal) - _validPushPoints[k];
            // robotEndPoint = robotEndPoint.Normalize() * (-OFF_SET);
            // robotEndPoint += _validPushPoints[k];
            cv::circle(_cameraImage, convertToCV(_validPushPoints[k]), INTERWHEEL_DISTANCE*SCALE/2.0f, cv::Scalar(0,0,0), -1);
        }
    }
}


/**
 * @brief Server WAIT state function
 * @param pcBox The argos box entity
 * @param _boxGoal The goal location of the box
 * @param _clientcount The amount of clients connected to the server
 * @param _clientConnections Each individual instance of protocol
 * @param _debug Parameter to write things to the log
*/
bool serverWaitState(argos::CBoxEntity *pcBox, argos::CVector3 _boxGoal, int _clientcount, std::vector<protocol> &_clientConnections, double threshold, bool _debug = false)
{
    bool _inRange;
    argos::CVector3 boxOrigin = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;
    argos::Real distanceToGoal = argos::Distance(_boxGoal, boxOrigin);

    if(_debug)
    {
        argos::LOG << "dist2goal: " << distanceToGoal << std::endl;
    }
    
    if(distanceToGoal < threshold)
    {
        if(_debug)
            argos::LOG << "IN GOAL RANGE\n";
        for (size_t i = 0; i < _clientcount; i++)
        {
            if(_clientConnections[i].send("STOP"));
            if(_debug)
                argos::LOG << "----------------------send message-------------------------------- \n";
        }
        _inRange = true;
    }
    else
        _inRange = false;

    return _inRange;
}

