#include "planner.h"

using namespace argos;

planner::planner(){

}
planner::~planner(){}

planner::cPositions planner::findCPositions(CBoxEntity* mBox)
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
Real planner::projection(V &v1, V &v2)
{
    //return v1.DotProduct(v2)/(sqrt(pow(v2.GetX(),2)+pow(v2.GetY(),2)));
    return v1.DotProduct(v2)/(abs(v2.GetX()+v2.GetY()));    //Absolute value is taken since we're only interested in the sign of the value.
}
std::vector<CVector3> planner::findPushPoints(CBoxEntity* mBox, CVector3 goalPoint)
{
    std::vector<CVector3> validPushPoints;
    planner::cPositions c = planner::findCPositions(mBox);

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
        if(projection(dvBox, c1Vecs[i]) < 0 && !c1Found)
        {
            validPushPoints.push_back(c.c1);
            c1Found = 1;
            if(debug)
                LOG << "Projections corner 1: " << projection(dvBox, c1Vecs[i]) << std::endl;
        }
        if(projection(dvBox, c2Vecs[i]) < 0 && !c2Found)
        {
            validPushPoints.push_back(c.c2);
            c2Found = 1;
            if(debug)
                LOG << "Projections corner 2: " << projection(dvBox, c2Vecs[i]) << std::endl;
        }
        if(projection(dvBox, c3Vecs[i]) < 0 && !c3Found)
        {
            validPushPoints.push_back(c.c3);
            c3Found = 1;
            if(debug)
                LOG << "Projections corner 3: " << projection(dvBox, c3Vecs[i]) << std::endl;
        }
        if(projection(dvBox, c4Vecs[i]) < 0 && !c4Found)
        {
            validPushPoints.push_back(c.c4);
            c4Found = 1;
            if(debug)
                LOG << "Projections corner 4: " << projection(dvBox, c4Vecs[i]) << std::endl;
        }
    }

    return validPushPoints;
}

void planner::wavefront(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal)
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
         cv::Point(1, -1),
    }};
    //Start wavefront fra goal, og brug neighbour til at tegne ud til alle sider og add de nye "farvede" punkter til en queue, og gør det samme for dem osv.
    cv::namedWindow("wavefront", cv::WINDOW_NORMAL);
    //cv::resizeWindow("wavefront", 500, 500);

    int map_size = 20;
    int scale = map_size/20;
    int animation_speed = 400;

    cv::Mat test(map_size, map_size, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat map_brushfire;

    test.at<cv::Vec3b>(cv::Point(robot.GetX()+5, robot.GetY()+5)) = {0, 0, 255};
    test.at<cv::Vec3b>(cv::Point(goal.GetX()+15, goal.GetY()+15)) = {0, 255, 0};

    // //Generate terrain/obstacles
    // cv::rectangle(test, cv::Point(2,2)*scale, cv::Point(5,8)*scale,cv::Scalar(0,0,0), -1);
    // cv::rectangle(test, cv::Point(6,10)*scale, cv::Point(10,12)*scale,cv::Scalar(0,0,0), -1);

    cv::resizeWindow("wavefront", 700, 700);
    cv::imshow("wavefront", test);

}