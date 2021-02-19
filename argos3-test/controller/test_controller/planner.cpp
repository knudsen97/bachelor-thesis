#include "planner.h"
#include "queue"
#include "algorithm"

using namespace argos;

#define MAX_USHORT 65535
planner::planner(){

}
planner::~planner(){}

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
    Real xSize = mBox->GetSize().GetX();
    Real ySize = mBox->GetSize().GetY();    

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

//Used to calculate the projection in findPushPoints() function:
template<class V>
Real planner::Projection(V &v1, V &v2)
{
    //return v1.DotProduct(v2)/(sqrt(pow(v2.GetX(),2)+pow(v2.GetY(),2)));
    return v1.DotProduct(v2)/(abs(v2.GetX()+v2.GetY()));    //Absolute value is taken since we're only interested in the sign of the value.
}
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
    for(size_t i = 0; i<2; i++) //TODO: DENNE LØKKE KAN GODT TAGE DET SAMME PUNKT 2 GANGE!!!!!!
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

ushort PixelVal(cv::Mat &map, cv::Point point)
{
    return map.at<ushort>(point);
}

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

    cv::namedWindow("wavefront", cv::WINDOW_NORMAL);
    cv::namedWindow("gray wavefront", cv::WINDOW_NORMAL);

    int map_size = 20;
    int scale = map_size/20;
    int animation_speed = 400;

    //I created my own map here, this should be the input parameter "map" in the future!
    cv::Mat grayMap;
    cv::Mat test(map_size, map_size, CV_8UC3, cv::Scalar(255,255,255));

    // //Generate terrain/obstacles and insert robot start and end goal.
    // cv::rectangle(test, cv::Point(2,2)*scale, cv::Point(5,8)*scale,cv::Scalar(0,0,0), -1);
    cv::rectangle(test, cv::Point(6,10)*scale, cv::Point(10,12)*scale,cv::Scalar(0,0,0), -1);
    cv::rectangle(test, cv::Point(0,0)*scale, cv::Point(map_size-1,map_size-1)*scale, cv::Scalar(0,0,0));
    cv::cvtColor(test, grayMap, cv::COLOR_BGR2GRAY);
    grayMap.convertTo(grayMap, CV_16UC1, 257.0f);

    test.at<cv::Vec3b>(cv::Point(robot.GetX()+5, robot.GetY()+5)) = {0, 0, 255};
    test.at<cv::Vec3b>(cv::Point(goal.GetX()+15, goal.GetY()+15)) = {0, 200, 0};

    // cv::resizeWindow("gray wavefront", 700, 700);
    // cv::imshow("gray wavefront", grayMap);
    cv::resizeWindow("wavefront", 700, 700);
    cv::imshow("wavefront", test);
    //cv::waitKey(0);

    //Start wavefront:
    bool pixelChange = true;
    ushort waveColor = MAX_USHORT-1;  
    std::vector<cv::Point> explorer, explorerColour;
    explorer.push_back({goal.GetX()+15, goal.GetY()+15});
    while(pixelChange)
    {
        pixelChange = false;
        for(cv::Point e : explorer)
        {
            for(int j = 0; j < neighbours.size(); j++)
            {
                if(PixelVal(grayMap, e + neighbours[j]) == MAX_USHORT && e + neighbours[j] != cv::Point(15,15)) //check if the pixel is white
                {
                    //cv::waitKey(0);
                    grayMap.at<ushort>(e + neighbours[j]) = waveColor;
                    pixelChange = true;
                    if(e.x + neighbours[j].x >= 0 && e.y + neighbours[j].y >= 0)
                        explorerColour.push_back(e + neighbours[j]);
                }
            }
        }
        waveColor -= 1;
        explorer = explorerColour;
        explorerColour.clear();
        cv::imshow("wavefront", test);
        //cv::imshow("gray wavefront", grayMap);
        //cv::waitKey(animation_speed);
    }

    test.copyTo(this->map);
    return grayMap;
}

std::vector<cv::Point> planner::Pathfinder(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal)
{
    cv::Mat grayMap;
    map.copyTo(grayMap);

    //cv::waitKey(0);
    // cv::namedWindow("gray wavefront", cv::WINDOW_NORMAL);
    // cv::imshow("gray wavefront", grayMap);

    cv::namedWindow("wavefront", cv::WINDOW_NORMAL);
    cv::imshow("wavefront", this->map);

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

    std::vector<cv::Point> goalPath;
    cv::Point traverse;
    cv::Point start = {robot.GetX()+5, robot.GetY()+5};
    cv::Point goalLocation = {goal.GetX()+15, goal.GetY()+15};
    traverse = start;
    int animationSpeed = 400;

    cv::Point PH = traverse + neighbours[0];
    int idx = 0;
    while(traverse != goalLocation)
    {
        for(size_t i = 1; i < neighbours.size()-1; i++)
        {
            if(PixelVal(grayMap, PH) < PixelVal(grayMap, traverse + neighbours[i]))
            {
                PH = traverse + neighbours[i];
                idx = i;
            }
            // if(i<neighbours.size()-1)
            //     traverse += neighbours[i];
        }
        traverse += neighbours[idx];
        this->map.at<cv::Vec3b>(PH) = cv::Vec3b(0,255,0);
        cv::imshow("wavefront", this->map);
        cv::waitKey(0);

    }

    return goalPath;
}