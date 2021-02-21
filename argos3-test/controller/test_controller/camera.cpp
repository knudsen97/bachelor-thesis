#include "camera.h"
#include <cmath>

#define IMAGESIZE 700
#define WINDOWSIZE 500
#define ARENASIZE 3
#define SCALE (IMAGESIZE/ARENASIZE)

using namespace argos;

void camera::PreStep()
{
    camera::ClearPlot();
    /**
     * These for-loop is written after this example 
     * https://www.argos-sim.info/code_page.php?path=examples/loop_functions/trajectory_loop_functions/trajectory_loop_functions.cpp&lang=cpp
    */
    CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
    for (CSpace::TMapPerType::iterator i = boxMap.begin(); i != boxMap.end(); ++i)
    {
        CBoxEntity* pBox = any_cast<CBoxEntity*>(i->second);
        camera::AddBox(pBox);
    }

    CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("foot-bot");
    for (CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
    {
        CFootBotEntity* fBot = any_cast<CFootBotEntity*>(i->second);
        camera::AddRobotPosition(fBot->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
}

camera::polygon findCPositions(argos::CBoxEntity* mBox)
{
    camera::polygon polygon;
    //Find the box' origin/center of mass:
    cv::Point2d origin = {mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()};
    
    //Get the Orientation of the box:
    argos::CRadians xAngle, yAngle, zAngle;
    mBox->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
    float theta = -xAngle.GetAbsoluteValue() + M_PI_2;
    
    //Get box dimensions to calculate offset:
    float xSize = mBox->GetSize().GetX();
    float ySize = mBox->GetSize().GetY();

    //TOP RIGHT CORNER:
    float c1x = origin.x + ((ySize / 2) * cos(theta)) - ((xSize / 2) * sin(theta));
    float c1y = origin.y + ((ySize / 2) * sin(theta)) + ((xSize / 2) * cos(theta));

    //TOP LEFT CORNER:
    float c2x  = origin.x - ((ySize / 2) * cos(theta)) - ((xSize / 2) * sin(theta));
    float c2y  = origin.y - ((ySize / 2) * sin(theta)) + ((xSize / 2) * cos(theta));

    //BOTTOM LEFT CORNER:
    float c3x = origin.x - ((ySize / 2) * cos(theta)) + ((xSize / 2) * sin(theta));
    float c3y = origin.y - ((ySize / 2) * sin(theta)) - ((xSize / 2) * cos(theta));

    //BOTTOM RIGHT CORNER:
    float c4x = origin.x + ((ySize / 2) * cos(theta)) + ((xSize / 2) * sin(theta));
    float c4y = origin.y + ((ySize / 2) * sin(theta)) - ((xSize / 2) * cos(theta));

    polygon.corners.push_back({(c1x)*SCALE, (c1y)*SCALE});
    polygon.corners.push_back({(c2x)*SCALE, (c2y)*SCALE});
    polygon.corners.push_back({(c3x)*SCALE, (c3y)*SCALE});
    polygon.corners.push_back({(c4x)*SCALE, (c4y)*SCALE});

    return polygon;
}


camera::camera()
{
    std::string name;
    name.append(camera::windowNameTemp);
    name.append(" ");
    name.append(std::to_string(camera::windowCounter));
    cv::resizeWindow(name, WINDOWSIZE, WINDOWSIZE);
    camera::windowName = name;
    camera::windowCounter++;
    frame = emptyFrame.clone();
}

camera::camera(std::string name)
{
    cv::resizeWindow(name, WINDOWSIZE, WINDOWSIZE);
    camera::windowName = name;
    frame = emptyFrame.clone();
}

camera::~camera(){}

void camera::AddBox(argos::CBoxEntity* box)
{
    camera::polygon polygon;
    polygon = findCPositions(box);
    cv::fillPoly(frame, polygon.corners, cv::Scalar(0,0,0));
    // for (size_t i = 0; i < polygon.corners.size(); i++)
    // {
    //     argos::LOG << "corner " << std::to_string(i) << ": " << polygon.corners[i]/(SCALE/100) << "\n";
    // }     
}

/**
 * Draw robot into the frame
 * @param robot is the robot coordiante
 * @param robotRadius is the robot radius in meter 
*/
void camera::AddRobotPosition(argos::CVector3 robot, float robotRadius)
{
    cv::Point robot_position(robot.GetX()*SCALE, robot.GetY()*SCALE);
    cv::circle(frame, robot_position, robotRadius * SCALE, cv::Scalar(255, 255, 000), 1);
    // argos::LOG << "robot position: " << robot_position/(SCALE/100) << "\n";
}

/**
 * Plots the frame. The frame is fliped in the x axis to match argos.
 * OpenCV use left hand coordinate system while argos uses right hand coordinate system
*/
void camera::Plot() //shows if fliped for visual purpose
{
    cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);
    cv::Mat frameToPlot;
    GetPlot(frameToPlot);
    cv::flip(frameToPlot, frameToPlot, 0);
    if(frameToPlot.cols > 0)
        cv::imshow(this->windowName, frameToPlot);
    
}

/**
 * Gets the frame either by returning it or giving the function a output matrix.
*/
cv::Mat camera::GetPlot() //get non-fliped to mach argos3 coordinates 
{
    cv::Mat retFrame = frame.clone();
    return retFrame;
}

/**
 * Gets the frame either by returning it or giving the function a output matrix.
 * @param outputframe The matrix where the frame will be saved to
*/
void camera::GetPlot(cv::Mat& outputFrame) //get non-fliped to mach argos3 coordinates 
{
    outputFrame = frame.clone();
}


void camera::ClearPlot()
{
    camera::frame = camera::emptyFrame.clone();
}


size_t camera::windowCounter = 0;
cv::Mat camera::emptyFrame(IMAGESIZE,IMAGESIZE, CV_8UC3, cv::Scalar(255,255,255));
cv::Mat camera::frame = camera::emptyFrame.clone();

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(camera, "camera_loop_function");

/****************************************/
/****************************************/
