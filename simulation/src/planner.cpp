#include "../inc/planner.h"

using namespace argos;


planner::planner(){}
planner::~planner(){}

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
 * Finds absolute value of projection between two vectors
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
 * Find eligible points on a CBoxEntity box to push from given a goal position.
 * @param mBox is the box entity in argos
 * @param goalPoint is the goal position
*/
std::vector<CVector3> planner::FindPushPoints(CBoxEntity* mBox, CVector3 goalPoint)
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

    //Create vector from box to goal:
    CVector3 dvBox = goalPoint - mBox->GetEmbodiedEntity().GetOriginAnchor().Position;
    //LOG << "dvBox: " << dvBox << std::endl;

    //Find pushing points by projecting dvBox vector with each corners vectors. If atleast 1 projection is negative it is a valid point.
    bool debug = false;
    bool c1Found=0, c2Found=0, c3Found=0, c4Found=0;
    for(size_t i = 0; i<2; i++) //TODO: DENNE LØKKE KAN GODT TAGE DET SAMME PUNKT 2 GANGE!!!!!! //Tror det er fikset nu. Spaghetti løsning
    {
        if(Projection(dvBox, c1Vecs[i]) < 0 && !c1Found)
        {
            validPushPoints.push_back(c.c1);
            c1Found = 1;
            if(debug)
                LOG << "Projections corner 1: " << Projection(dvBox, c1Vecs[i]) << std::endl;
        }
        if(Projection(dvBox, c2Vecs[i]) < 0 && !c2Found)
        {
            validPushPoints.push_back(c.c2);
            c2Found = 1;
            if(debug)
                LOG << "Projections corner 2: " << Projection(dvBox, c2Vecs[i]) << std::endl;
        }
        if(Projection(dvBox, c3Vecs[i]) < 0 && !c3Found)
        {
            validPushPoints.push_back(c.c3);
            c3Found = 1;
            if(debug)
                LOG << "Projections corner 3: " << Projection(dvBox, c3Vecs[i]) << std::endl;
        }
        if(Projection(dvBox, c4Vecs[i]) < 0 && !c4Found)
        {
            validPushPoints.push_back(c.c4);
            c4Found = 1;
            if(debug)
                LOG << "Projections corner 4: " << Projection(dvBox, c4Vecs[i]) << std::endl;
        }
    }

    return validPushPoints;
}

/**
 * Finds the value of a pixel given a gray-scale map and a point.
 * @param map is a cv::Mat object of a given map
 * @param point is coordinate of a pixel
*/
ushort GrayPixelVal(cv::Mat &map, cv::Point point)
{
    return map.at<ushort>(point);
}


/**
 * Generates a wavefront starting from the goal location
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

    //Define a kernel and erode the map inorder to not get close to obstacles
    cv::Mat kernel = cv::Mat(OFF_SET*SCALE, OFF_SET*SCALE, CV_8UC1);
    cv::erode(grayMap, grayMap, kernel);
    grayMap.copyTo(grayMapCopy);

    //cv::imshow("gray wavefront", grayMap);
    // cv::imshow("wavefront", map);
    //cv::waitKey(0);

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
    // cv::imshow("wavefront", map);
    //cv::waitKey(0);

    // map.copyTo(this->map);

    return grayMap;
}


/**
 * Finds a path from the robot's start location to it's box' corner subgoal
 * @param grayMap is a cv::Mat object of a given map in gray-scale
 * @param robot is the location of the robot
 * @param goal is the goal location being a corner of the box.
*/
std::vector<cv::Point> planner::Pathfinder(cv::Mat &grayMap, argos::CVector3 &robot, argos::CVector3 &goal)
{

    std::array<cv::Point, 8> neighbours =
    {{
         cv::Point(-1, 1),
         cv::Point(0,  1),
         cv::Point(1,  1),
         cv::Point(1,  0),
         cv::Point(1,  -1),
         cv::Point(0,  -1),
         cv::Point(-1, -1),
         cv::Point(-1, 0)
    }};

    std::vector<cv::Point> goalPath, postProcessPath;
    cv::Point traverse;

    //Convert argos vector to cv::Point:
    cv::Point start(robot.GetX()*SCALE, robot.GetY()*SCALE);
    cv::Point goalLocation(goal.GetX()*SCALE, goal.GetY()*SCALE);

    traverse = start;
    int animationSpeed = 400;

    cv::Point PH = traverse + neighbours[0];
    int idx = 0, prevIdx = 0;    //To keep track of which neighbour was used in order to illustrate
    bool foundGoal = 0;

    while(!foundGoal)//GrayPixelVal(grayMap, traverse) != 0)//< MAX_USHORT-1)
    {
        prevIdx = idx;
        for(size_t i = 0; i < neighbours.size(); i++)
        {
            //std::cout << i <<"'th goal pixel val: " << (GrayPixelVal(grayMap, traverse + neighbours[i])) << std::endl;
            if(GrayPixelVal(grayMap, PH) <= GrayPixelVal(grayMap, traverse + neighbours[i])) 
            {
                PH = traverse + neighbours[i];
                idx = i;
            }
            else if (GrayPixelVal(grayMap, traverse + neighbours[i]) >= 65533)
            {
                foundGoal = 1;
                goalPath.push_back(traverse + neighbours[i]);
                // cv::circle(this->map, traverse + neighbours[i], 3, cv::Scalar(0,255,255), -1);    //Illustration purpose
                break;
            }
        }

        traverse += neighbours[idx];
        
        if(prevIdx != idx)
        {
            // cv::circle(this->map, PH, 3, cv::Scalar(0,255,255), -1);    //Illustration purpose
            goalPath.push_back(PH);
        }
    }

    //To show the point where the box needs to go:
    cv::circle(map, cv::Point(2*SCALE, 2*SCALE), 3, cv::Scalar(255,0,0),-1);
    // cv::imshow("wavefront", this->map);

    // for(auto goal : goalPath)
    //     std::cout << goal << std::endl;

    return goalPath;
}

/**
 * Optimizes the path generated from wavefront by deleting unneccesary points in a vector
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

/**
 * Checks if there are any obstacles between two points in a cv::Mat object
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

/**
 * This function is a helper function for planner::push to calculate the robots position after pushing the object.
 * @brief Translate a point with an orientation and a distance
 * @param point The point to translate
 * @param orientation The direction to translate the point in radians
 * @param distance The distance to translate the point in said direction
*/
argos::CVector3 translate(argos::CVector3 point, argos::CRadians orientation, argos::Real distance)
{
    argos::CVector3 returnPoint;

    returnPoint.SetX(point.GetX() + argos::Sin(orientation)*distance);
    returnPoint.SetY(point.GetY() + argos::Cos(orientation)*distance);
    returnPoint.SetZ(point.GetZ());
    return returnPoint;
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
    argos::CRadians orientation = argos::ATan2(goalPoint.GetY() - boxOrigin.GetY(), goalPoint.GetX() - boxOrigin.GetX());
    return translate(currentPoint, orientation, distance);
}
