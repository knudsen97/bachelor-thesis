#include "../inc/planner.h"

using namespace argos;


planner::planner(){}
planner::~planner(){}


/**
 * @brief This function converts a ArgOS CVector3 to an OpenCV Point.
 * @param arg An ArgOS CVector3
**/
cv::Point toCV(argos::CVector3 arg)
{
   return cv::Point(arg.GetX()*SCALE, arg.GetY()*SCALE);
}


/**
 * Finds corner position of a CBoxEntity
 * @param mBox is the argos box entity
*/
planner::cPositions planner::FindCPositions(CBoxEntity* mBox)
{

    //Find the box' origin/center of mass:
    CVector2 origin = {mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()};
    //Get the Orientation of the box:
    CRadians xAngle, yAngle, zAngle;
    mBox->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
    Real theta = xAngle.GetValue();          //cos and sin does'nt take degrees but radians*180/PI;     //Radians to degrees
    //LOG << "Angle: " << theta << std::endl;

    //Get box dimensions to calculate offset:
    Real xSize = mBox->GetSize().GetX();// + OFF_SET;
    Real ySize = mBox->GetSize().GetY();// + OFF_SET;    

    //TOP RIGHT CORNER:
    Real c1x = origin.GetX() + ((xSize / 2) * cos(theta)) - ((ySize / 2) * sin(theta));
    Real c1y = origin.GetY() + ((xSize / 2) * sin(theta)) + ((ySize / 2) * cos(theta));

    //TOP LEFT CORNER:
    Real c2x  = origin.GetX() - ((xSize / 2) * cos(theta)) - ((ySize / 2) * sin(theta));
    Real c2y  = origin.GetY() - ((xSize / 2) * sin(theta)) + ((ySize / 2) * cos(theta));

    //BOTTOM LEFT CORNER:
    Real c3x = origin.GetX() - ((xSize / 2) * cos(theta)) + ((ySize / 2) * sin(theta));
    Real c3y = origin.GetY() - ((xSize / 2) * sin(theta)) - ((ySize / 2) * cos(theta));

    //BOTTOM RIGHT CORNER:
    Real c4x = origin.GetX() + ((xSize / 2) * cos(theta)) + ((ySize / 2) * sin(theta));
    Real c4y = origin.GetY() + ((xSize / 2) * sin(theta)) - ((ySize / 2) * cos(theta));

    planner::cPositions c;
    c.c1 = {c1x, c1y, 0};
    c.c2 = {c2x, c2y, 0};
    c.c3 = {c3x, c3y, 0};
    c.c4 = {c4x, c4y, 0};

    return c;
}


/**
 * Finds corner position an irregular shaped object of a CBoxEntity
 * @param mBox is the argos box entity
*/
std::vector<argos::CVector3> planner::FindPolygonCorners(argos::CPrototypeEntity* mObject)
{
    /* Get a map of the current box plotted: */
    camera cam;
    cv::Mat objectMap = cam.PlotBox(mObject);
    cv::cvtColor(objectMap, objectMap, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(objectMap, objectMap, cv::Size(9, 9), 2, 2 );

    /* Parameter definitions for goodFeaturesToTrack openCV function: */
    std::vector<cv::Point2f> corners; // To store answers
    int maxCorners = 10; 
    double qLevel = 0.1;
    double minDist = 10;
    int blockSize = 3, gradientSize = 3;
    bool useHarrisDetect = false;
    double k = 0.04;

    cv::goodFeaturesToTrack(objectMap,
                            corners,
                            maxCorners,
                            qLevel,
                            minDist,
                            cv::Mat(),
                            blockSize,
                            gradientSize,
                            useHarrisDetect,
                            k );
    // std::cout << "COrners detected: " << corners.size() << std::endl;

    // for(auto corner : corners)
    // {
    //     cv::circle(objectMap, corner, 5, 0, -1);
    // }
    // cv::imshow("test", objectMap);
    // cv::waitKey(0);

    /* Convert the corners found to argos coordinates */
    std::vector<argos::CVector3> corners_;
    for(auto corner : corners)
    {
        corners_.push_back(argos::CVector3(corner.x/SCALE, corner.y/SCALE, 0));
    }

    return corners_;
}

/**
 * Finds corner position an irregular shaped object of a CBoxEntity
 * @param mBox is the argos box entity
*/
std::vector<argos::CVector3> planner::FindPolygonCorners(argos::CBoxEntity* mBox)
{
    /* Get a map of the current box plotted: */
    camera cam;
    cv::Mat objectMap = cam.PlotBox(mBox);
    cv::cvtColor(objectMap, objectMap, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(objectMap, objectMap, cv::Size(9, 9), 2, 2 );

    /* Parameter definitions for goodFeaturesToTrack openCV function: */
    std::vector<cv::Point2f> corners; // To store answers
    int maxCorners = 10; 
    double qLevel = 0.1;
    double minDist = 10;
    int blockSize = 3, gradientSize = 3;
    bool useHarrisDetect = false;
    double k = 0.04;

    cv::goodFeaturesToTrack(objectMap,
                            corners,
                            maxCorners,
                            qLevel,
                            minDist,
                            cv::Mat(),
                            blockSize,
                            gradientSize,
                            useHarrisDetect,
                            k );
    // std::cout << "COrners detected: " << corners.size() << std::endl;

    // for(auto corner : corners)
    // {
    //     cv::circle(objectMap, corner, 5, 0, -1);
    // }
    // cv::imshow("test", objectMap);
    // cv::waitKey(0);

    /* Convert the corners found to argos coordinates */
    std::vector<argos::CVector3> corners_;
    for(auto corner : corners)
    {
        corners_.push_back(argos::CVector3(corner.x/SCALE, corner.y/SCALE, 0));
    }

    return corners_;
}


/**
 * Find eligible points on a CBoxEntity box to push from given a goal position.
 * @param mBox is the box entity in argos
 * @param goalPoint is the goal position
*/
std::vector<CVector3> planner::FindPushPointsBox(CBoxEntity* mBox, CVector3 goalPoint)
{
    std::vector<CVector3> validPushPoints;
    planner::cPositions c = planner::FindCPositions(mBox);

    //Find corner vectors:
    std::array<CVector3,2> c1Vecs, c2Vecs, c3Vecs, c4Vecs;
    c1Vecs = {c.c1 - c.c2, c.c1 - c.c4};
    c2Vecs = {c.c2 - c.c1, c.c2 - c.c3};
    c3Vecs = {c.c3 - c.c2, c.c3 - c.c4};
    c4Vecs = {c.c4 - c.c1, c.c4 - c.c3};

    //For debugging:
    // LOG << "c1Vecs[0]: " << c1Vecs[0] << std::endl;
    // LOG << "c1Vecs[1]: " << c1Vecs[1] << std::endl;

    // LOG << "c2Vecs[0]: " << c2Vecs[0] << std::endl;
    // LOG << "c2Vecs[1]: " << c2Vecs[1] << std::endl;

    // LOG << "c3Vecs[0]: " << c3Vecs[0] << std::endl;
    // LOG << "c3Vecs[1]: " << c3Vecs[1] << std::endl;

    // LOG << "c4Vecs[0]: " << c4Vecs[0] << std::endl;
    // LOG << "c4Vecs[1]: " << c4Vecs[1] << std::endl;

    /*Create vector from box to goal:*/
    CVector3 dvBox = goalPoint - mBox->GetEmbodiedEntity().GetOriginAnchor().Position;
    //LOG << "dvBox: " << dvBox << std::endl;

    /*Find pushing points by projecting dvBox vector with each corners vectors. If atleast 1 projection is negative it is a valid point.*/
    bool debug = false;
    bool c1Found=0, c2Found=0, c3Found=0, c4Found=0;
    for(size_t i = 0; i<2; i++)
    {
        if(Projection(dvBox, c1Vecs[i]) < 0 && !c1Found)
        {
            c.c1 = offsetPoint(mBox, c.c1, goalPoint);
            validPushPoints.push_back(c.c1);
            c1Found = 1;
            if(debug)
                LOG << "Projections corner 1: " << Projection(dvBox, c1Vecs[i]) << std::endl;
        }
        if(Projection(dvBox, c2Vecs[i]) < 0 && !c2Found)
        {
            c.c2 = offsetPoint(mBox, c.c2, goalPoint);
            validPushPoints.push_back(c.c2);
            c2Found = 1;
            if(debug)
                LOG << "Projections corner 2: " << Projection(dvBox, c2Vecs[i]) << std::endl;
        }
        if(Projection(dvBox, c3Vecs[i]) < 0 && !c3Found)
        {
            c.c3 = offsetPoint(mBox, c.c3, goalPoint);
            validPushPoints.push_back(c.c3);
            c3Found = 1;
            if(debug)
                LOG << "Projections corner 3: " << Projection(dvBox, c3Vecs[i]) << std::endl;
        }
        if(Projection(dvBox, c4Vecs[i]) < 0 && !c4Found)
        {
            c.c4 = offsetPoint(mBox, c.c4, goalPoint);
            validPushPoints.push_back(c.c4);
            c4Found = 1;
            if(debug)
                LOG << "Projections corner 4: " << Projection(dvBox, c4Vecs[i]) << std::endl;
        }
    }

    return validPushPoints;
}

/**
 * Find eligible points on a irregular shaped object to push from given a goal position.
 * @param mBox is the box entity in argos
 * @param goalPoint is the goal position
*/
std::vector<argos::CVector3> planner::FindPushPointsIrregular(argos::CBoxEntity* mBox, argos::CVector3 goalPoint)
{
    std::vector<CVector3> validPushPoints, tempPoints, corners;
    corners = FindPolygonCorners(mBox);


    /* Find off set corner location */
    for(auto corner : corners)
        tempPoints.push_back(offsetPoint(mBox, corner, goalPoint));

    /* Generate cv::Mat to check for collision */
    camera cam;
    cv::Mat objectMap = cam.PlotBox(mBox);
    cv::cvtColor(objectMap, objectMap, cv::COLOR_BGR2GRAY);
    cv::Mat robotMap = objectMap.clone();
    robotMap.setTo(255);
    //cv::Mat copy = 

    cv::Mat nor;
    bool noIntersect = true;
    for(argos::CVector3 robotPosition : tempPoints)
    {   
        cv::circle(robotMap, toCV(robotPosition), INTERWHEEL_DISTANCE*SCALE, 0, -1);

        /* Debugging */
        // cv::circle(objectMap, toCV(robotPosition), INTERWHEEL_DISTANCE*SCALE, 0, -1);
        // cv::imshow("test", objectMap);
        // cv::waitKey(0);

        /* Check for intersections between robotMap and objectMap */
        cv::bitwise_or(robotMap, objectMap, nor);
        for(int row = 0; row < nor.rows && noIntersect; row++)
        {
            for(int col = 0; col < nor.cols && noIntersect; col++)
            {
                if(nor.at<uchar>(row,col) != 255)
                    noIntersect = false;
            }
        }
        if(noIntersect)
        {
            validPushPoints.push_back(robotPosition);
        }
        noIntersect = true;
        robotMap.setTo(255);
        nor.setTo(255);
    }

    // std::cout << "validPushPoints: " << validPushPoints.size() << std::endl;
    // cv::waitKey(0);


    return validPushPoints;
}

/**
 * Find eligible points on a irregular shaped object to push from given a goal position.
 * @param mObject is the prototype entity in argos
 * @param goalPoint is the goal position
*/
std::vector<argos::CVector3> planner::FindPushPointsIrregular(argos::CPrototypeEntity* mObject, argos::CVector3 goalPoint)
{
    std::vector<CVector3> validPushPoints, tempPoints, corners;
    corners = FindPolygonCorners(mObject);


    /* Find off set corner location */
    for(auto corner : corners)
        tempPoints.push_back(offsetPoint(mObject, corner, goalPoint));

    /* Generate cv::Mat to check for collision */
    camera cam;
    cv::Mat objectMap = cam.PlotBox(mObject);
    cv::cvtColor(objectMap, objectMap, cv::COLOR_BGR2GRAY);
    cv::Mat robotMap = objectMap.clone();
    robotMap.setTo(255);
    //cv::Mat copy = 

    cv::Mat nor;
    bool noIntersect = true;
    for(argos::CVector3 robotPosition : tempPoints)
    {   
        cv::circle(robotMap, toCV(robotPosition), INTERWHEEL_DISTANCE*SCALE, 0, -1);

        /* Debugging */
        // cv::circle(objectMap, toCV(robotPosition), INTERWHEEL_DISTANCE*SCALE, 0, -1);
        // cv::imshow("test", objectMap);
        // cv::waitKey(0);

        /* Check for intersections between robotMap and objectMap */
        cv::bitwise_or(robotMap, objectMap, nor);
        for(int row = 0; row < nor.rows && noIntersect; row++)
        {
            for(int col = 0; col < nor.cols && noIntersect; col++)
            {
                if(nor.at<uchar>(row,col) != 255)
                    noIntersect = false;
            }
        }
        if(noIntersect)
        {
            validPushPoints.push_back(robotPosition);
        }
        noIntersect = true;
        robotMap.setTo(255);
        nor.setTo(255);
    }

    // std::cout << "validPushPoints: " << validPushPoints.size() << std::endl;
    // cv::waitKey(0);


    return validPushPoints;
}


/**
 * @brief Generates a wavefront starting from the goal location
 * @param map is a cv::Mat object of a given map
 * @param robot is the robots start location
 * @param goal is the location of a chosen corner of the box to be pushed
*/
cv::Mat planner::Wavefront(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal)
{

    std::array<cv::Point, 8> neighbours =
    {{
         cv::Point(-1, 1),
         cv::Point(0, 1),
         cv::Point(1, 1),
         cv::Point(-1, 0),
         cv::Point(1,0),
         cv::Point(-1, -1),
         cv::Point(0, -1),
         cv::Point(1, -1)
    }};

    // cv::namedWindow("wavefront", cv::WINDOW_NORMAL);
    // cv::resizeWindow("wavefront", WINDOWSIZE, WINDOWSIZE);
    //Whenever I can get map from camera class use this:
    cv::Mat grayMap;
    cv::cvtColor(map, grayMap, cv::COLOR_BGR2GRAY);
    grayMap.convertTo(grayMap, CV_16UC1, 257.0f);
    
    // //Define a kernel and erode the map inorder to not get close to obstacles
    // cv::Mat kernel = cv::Mat(OFF_SET*SCALE, OFF_SET*SCALE, CV_8UC1);
    // cv::erode(grayMap, grayMap, kernel);
    // grayMap.copyTo(grayMapCopy);


    //cv::imshow("gray wavefront", grayMap);
    //cv::imshow("wavefront", map);
    //cv::waitKey(1);

    //Convert argos vector to cv::Point:
    cv::Point start(robot.GetX()*SCALE, robot.GetY()*SCALE);
    cv::Point goalLocation(goal.GetX()*SCALE, goal.GetY()*SCALE);

    // //Draw robot start and goal location
    // cv::circle(map, start, 5, cv::Scalar(0,0,255), -1);
    // cv::circle(map, goalLocation, 5, cv::Scalar(0,200,0), -1);
    //cv::waitKey(0);


    //Start wavefront:
    bool pixelChange = true;
    ushort calcColor = MAX_USHORT-1;  
    uchar illustrationColor = 255;
    std::vector<cv::Point> explorer, explorerColour;
    explorer.push_back(goalLocation);
    while(pixelChange)
    {
        pixelChange = false;
        for(cv::Point e : explorer)
        {
            for(int j = 0; j < neighbours.size(); j++)
            {
                if(GrayPixelVal(grayMap, e + neighbours[j]) == MAX_USHORT) //check if the pixel is white
                {
                    grayMap.at<ushort>(e + neighbours[j]) = calcColor;
                    pixelChange = true;
                    if(e.x + neighbours[j].x >= 0 && e.y + neighbours[j].y >= 0)
                        explorerColour.push_back(e + neighbours[j]);
                    // //Don't overdraw goal and start location for visualization:
                    // if(map.at<cv::Vec3b>(e+neighbours[j]) !=  cv::Vec3b(0,0,255) && map.at<cv::Vec3b>(e+neighbours[j]) !=  cv::Vec3b(0,200,0))//e + neighbours[j] != goalLocation && e + neighbours[j] != start)
                    //     map.at<cv::Vec3b>(e + neighbours[j]) = cv::Vec3b(illustrationColor, illustrationColor, illustrationColor); 
                }
            }
        }

        calcColor -= 1;
        illustrationColor -= 0.01;
        explorer = explorerColour;
        explorerColour.clear();


    }

    //cv::imshow("wavefront", grayMap);
    // cv::waitKey(1);
    // map.copyTo(this->map);
    //debugMap = grayMap.clone();
    return grayMap;
}

/**
 * @brief Finds a path from the robot's start location to it's box' corner subgoal
 * @param grayMap is a cv::Mat object of a given map in gray-scale
 * @param robot is the location of the robot
 * @param goal is the goal location being a corner of the box.
*/
std::vector<cv::Point> planner::Pathfinder(cv::Mat &grayMap, argos::CVector3 &robot, argos::CVector3 &goal)
{
    //debugMap = grayMap.clone();
    // std::array<cv::Point, 8> neighbours =
    // {{
    //      cv::Point(-1, 1),
    //      cv::Point(0,  1),
    //      cv::Point(1,  1),
    //      cv::Point(1,  0),
    //      cv::Point(1,  -1),
    //      cv::Point(0,  -1),
    //      cv::Point(-1, -1),
    //      cv::Point(-1, 0)
    // }};
    std::array<cv::Point, 8> neighbours =
    {{
        cv::Point(-1, 1),
        cv::Point(1,  1),
        cv::Point(1,  -1),
        cv::Point(-1, -1),
        cv::Point(0,  1),
        cv::Point(1,  0),
        cv::Point(0,  -1),
        cv::Point(-1, 0)
    }};

    std::vector<cv::Point> goalPath, postProcessPath;

    /*Convert argos vector to cv::Point:*/
    cv::Point start(robot.GetX()*SCALE, robot.GetY()*SCALE);
    cv::Point goalLocation(goal.GetX()*SCALE, goal.GetY()*SCALE);

    cv::Point traverse = start;
    cv::Point PH = traverse + neighbours[0];

    /*To keep track of which neighbour was used in order to illustrate*/
    int idx = 0, prevIdx = 0;   
    bool foundGoal = 0;
    int diff = 0;

    int spacing = 0;
    while(!foundGoal)
    {
        prevIdx = idx;
        for(size_t i = 0; i < neighbours.size(); i++)
        {      
            ushort nextPixelVal = GrayPixelVal(grayMap, traverse + neighbours[i]); 

            //debug = GrayPixelVal(grayMap, traverse + neighbours[i]);
            // debug = GrayPixelVal(grayMap, PH);
            if(GrayPixelVal(grayMap, PH) <= nextPixelVal && GrayPixelVal(grayMap, PH) != USHRT_MAX) 
            {
                PH = traverse + neighbours[i];
                idx = i;
                spacing++;
                //debug = GrayPixelVal(grayMap, PH);
                //cv::circle(debugMap, PH, 3, cv::Scalar(100,100,100), -1);
            }
            else if(nextPixelVal >= USHRT_MAX-2)
            {
                foundGoal = true;
                goalPath.push_back(traverse + neighbours[i]);
                //cv::circle(this->map, traverse + neighbours[i], 3, cv::Scalar(0,255,255), -1);    //Illustration purpose

                break;
            }
        }

        //argos::LOG << "test: " <<test << std::endl;
        if((prevIdx != idx && PH.x >=0 && PH.y >= 0) && spacing > 30)
        {
            spacing = 0;
            // cv::circle(this->map, PH, 3, cv::Scalar(0,255,255), -1);    //Illustration purpose
            //debug = PH.x;
            goalPath.push_back(PH);
        }

        traverse += neighbours[idx];

    }

    /*To show the point where the box needs to go:*/
    //cv::circle(map, cv::Point(2*SCALE, 2*SCALE), 3, cv::Scalar(255,0,0),-1);
    //cv::imshow("wavefront", grayMap);
    //cv::waitKey(10);

    // for(auto goal : goalPath)
    //     argos::LOG << goal << std::endl;
    

    return goalPath;
}

/**
 * Optimizes the path generated from wavefront by deleting unneccesary points in a vector. Is not currently in use
 * @brief not currently used
 * @param subGoals is the vector of goals
 */
std::vector<cv::Point> planner::PostProcessing(std::vector<cv::Point> &subGoals)
{
    for(size_t i = 0; subGoals.size()-2; i++)
    {
        if(planner::ValidLine(subGoals[i], subGoals[i+2], grayMapCopy))
        {
            subGoals.erase(
                std::remove_if(subGoals.begin(), subGoals.end(), [&](cv::Point const & point){
                    return point == subGoals[i+1];
                }),
                subGoals.end());
            // cv::circle(this->map, subGoals[i], 3, cv::Scalar(0,255,255), -1);
            // cv::imshow("wavefront", map);
            i = 0;
        }
    }


    return subGoals;
}


/***********************************************
******* AUXILIARY FUNCTIONS DEFINED HERE *******
************************************************/
/**
 * @brief Finds absolute value of projection between two vectors
 * @param v1 vector 1
 * @param v2 vector 2
*/
template<class V>
Real planner::Projection(V &v1, V &v2)
{
    //return v1.DotProduct(v2)/(sqrt(pow(v2.GetX(),2)+pow(v2.GetY(),2)));   //Real value
    return v1.DotProduct(v2)/(abs(v2.GetX()+v2.GetY()));    //Absolute value is taken since we're only interested in the sign of the value.
}

/**
 * This function calculate where the robot should be in order to push 
 * an object to where the object needs to be. This function assumes 
 * that there are enough robots to push the object so that the object 
 * does not rotate and only moving paralel to the robots trajectory 
 * while being pushed by the robot
 * 
 * @brief Calculates the endpoint for the robot in order to push the box to the box goal point.
 * @param mBox The box to push
 * @param currentPoint The robots current point
 * @param goalPoint The goalpoint for the box
*/
argos::CVector3 planner::push(argos::CBoxEntity* mBox, argos::CVector3 currentPoint, argos::CVector3 goalPoint)
{
    argos::CVector3 boxOrigin = mBox->GetEmbodiedEntity().GetOriginAnchor().Position;
    argos::Real distance = argos::Distance(goalPoint, boxOrigin);
    argos::CRadians orientation = argos::ATan2(goalPoint.GetX() - boxOrigin.GetX(), goalPoint.GetY() - boxOrigin.GetY());

    return translate(currentPoint, orientation, distance);
}
argos::CVector3 planner::push(argos::CPrototypeEntity* mBox, argos::CVector3 currentPoint, argos::CVector3 goalPoint)
{
    argos::CVector3 boxOrigin = mBox->GetEmbodiedEntity().GetOriginAnchor().Position;
    argos::Real distance = argos::Distance(goalPoint, boxOrigin);
    argos::CRadians orientation = argos::ATan2(goalPoint.GetX() - boxOrigin.GetX(), goalPoint.GetY() - boxOrigin.GetY());

    return translate(currentPoint, orientation, distance);
}

/**
 * This function is a helper function for planner::push to calculate the robots position after pushing the object.
 * @brief Translate a point with an orientation and a distance
 * @param point The point to translate
 * @param orientation The direction to translate the point in radians
 * @param distance The distance to translate the point in said direction
*/
argos::CVector3 planner::translate(argos::CVector3 point, argos::CRadians orientation, argos::Real distance)
{
    argos::CVector3 returnPoint;
    
    returnPoint.SetX(point.GetX() + argos::Sin(orientation)*distance);
    returnPoint.SetY(point.GetY() + argos::Cos(orientation)*distance);
    returnPoint.SetZ(point.GetZ());
    return returnPoint;
}

/**
 * @brief Finds the value of a pixel given a gray-scale map and a point.
 * @param map is a cv::Mat object of a given map
 * @param point is coordinate of a pixel
*/
ushort planner::GrayPixelVal(cv::Mat &map, cv::Point point)
{
    return map.at<ushort>(point);
}

/**
 * @brief Checks if there are any obstacles between two points in a cv::Mat object
 * @param A The first point
 * @param B The second point
 * @param img The map 
 */
bool planner::ValidLine(cv::Point A, cv::Point B, cv::Mat img)
{
    bool valid = true;
    cv::LineIterator it(img, A, B);
    for(size_t i = 1; i < it.count-1; i++, ++it)
    {
        valid = (*(const int*)* it == false) ? false : valid;
    }
    return valid;
}



argos::CVector3 planner::offsetPoint(argos::CBoxEntity* mBox, argos::CVector3 cornerLoc, argos::CVector3 goalPoint)
{
    argos::CVector3 robotEndPoint(0,0,0);
    robotEndPoint = push(mBox, cornerLoc, goalPoint) - cornerLoc;
    robotEndPoint = robotEndPoint.Normalize() * (-OFF_SET);
    robotEndPoint += cornerLoc;

    return robotEndPoint;
}

argos::CVector3 planner::offsetPoint(argos::CPrototypeEntity* mObject, argos::CVector3 cornerLoc, argos::CVector3 goalPoint)
{
    argos::CVector3 robotEndPoint(0,0,0);
    robotEndPoint = push(mObject, cornerLoc, goalPoint) - cornerLoc;
    robotEndPoint = robotEndPoint.Normalize() * (-OFF_SET);
    robotEndPoint += cornerLoc;

    return robotEndPoint;
}





