#include "../inc/camera.h"

using namespace argos;

void camera::step()
{
    camera::ClearPlot();
    /**
     * These for-loop is written after this example 
     * https://www.argos-sim.info/code_page.php?path=examples/loop_functions/trajectory_loop_functions/trajectory_loop_functions.cpp&lang=cpp
    */
    if (boxExist)
    {
        try
        {
            CSpace::TMapPerType& boxMap = GetSpace().GetEntitiesByType("box");
            for (CSpace::TMapPerType::iterator i = boxMap.begin(); i != boxMap.end(); ++i)
            {
                CBoxEntity* pBox = any_cast<CBoxEntity*>(i->second);
                // argos::LOG << "box id" << pBox->GetId() << '\n';
                // if(pBox->GetMass() > 0)
                //     camera::AddBox(pBox, {0, 0, 255});
                // else 
                    camera::AddBox(pBox);
            }
        }
        catch(const std::exception& e)
        {
            boxExist = false;
            argos::LOGERR << e.what() << '\n';
        }
    }
        
    

    if (objectExist)
    {
        try
        {
            CSpace::TMapPerType& objMap = GetSpace().GetEntitiesByType("prototype");
            for (CSpace::TMapPerType::iterator i = objMap.begin(); i != objMap.end(); ++i)
            {
                CPrototypeEntity* obj = any_cast<CPrototypeEntity*>(i->second);
                AddObject(obj);
            }
        }
        catch(const std::exception& e)
        {
            objectExist = false;
            argos::LOGERR << e.what() << '\n';
        }
    }
            
    if (epuckExist)
    {
        try
        {
            CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("e-puck");
            for (CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
            {
                CEPuckEntity* fBot = any_cast<CEPuckEntity*>(i->second);
                camera::AddRobotPosition(fBot->GetEmbodiedEntity().GetOriginAnchor().Position);
            }
        }
        catch(const std::exception& e)
        {
            epuckExist = false;
            argos::LOGERR << e.what() << '\n';
        }
    }

    if (footbotExist)
    {
        try
        {
            CSpace::TMapPerType& FBmap = GetSpace().GetEntitiesByType("foot-bot");
            for (CSpace::TMapPerType::iterator i = FBmap.begin(); i != FBmap.end(); ++i)
            {
                CEPuckEntity* fBot = any_cast<CEPuckEntity*>(i->second);
                camera::AddRobotPosition(fBot->GetEmbodiedEntity().GetOriginAnchor().Position);
            }
        }
        catch(const std::exception& e)
        {
            footbotExist = false;
            argos::LOGERR << e.what() << '\n';
        }
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
    float theta = xAngle.GetValue() + M_PI_2;
    
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

void translate(std::vector<cv::Point2i>& points, cv::Point2i destinationPoint)
{
    for(cv::Point2i& point : points)
        point += destinationPoint;
}
void rotate(std::vector<cv::Point2i>& points, const double theta)
{
    cv::Point2i p;
    for(cv::Point2i& point : points)
    {
        p.x = cos(theta)*point.x + -sin(theta)*point.y;
        p.y = sin(theta)*point.x +  cos(theta)*point.y;
        point = p;
    }

}

cv::Point2i cvtArgosOpencv(argos::CVector3 p)
{
    return cv::Point2i(p.GetX()*SCALE, p.GetY()*SCALE);
}

camera::polygon findObjectCorners(CPrototypeEntity* obj)
{
    camera::polygon polygon;
    argos::CPrototypeLinkEntity::TVector links;
    std::vector<argos::CVector3> edges;
    argos::CVector3 origin, edge;
    argos::CRadians theta_x, theta_y, theta_z;

    links = obj->GetLinkEquippedEntity().GetLinks();
    origin = obj->GetEmbodiedEntity().GetOriginAnchor().Position;
    obj->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(theta_z, theta_y, theta_x);

    // obj->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(theta_z, theta_y, theta_x);
    edges = links[0]->GetConvexHullPoints();
    for(size_t i = 0; i < edges.size()/2; i++)
    {
        polygon.corners.push_back( cvtArgosOpencv(edges[i]));
    }

    rotate(polygon.corners, theta_z.GetValue());
    translate(polygon.corners, cvtArgosOpencv(origin));
    return polygon;
}


camera::camera()
{
    //argos::LOG << "Make camera obj" << std::endl;

    std::string name;
    name.append(camera::windowNameTemp);
    name.append(" ");
    name.append(std::to_string(camera::windowCounter));
    cv::resizeWindow(name, WINDOWSIZE, WINDOWSIZE);
    camera::windowName = name;
    camera::windowCounter++;
    frame = emptyFrame.clone();
    objectContainer.push_back(this);
}
camera::camera(std::string name)
{
    //argos::LOG << "Make camera obj with arg" << std::endl;
    cv::resizeWindow(name, WINDOWSIZE, WINDOWSIZE);
    camera::windowName = name;
    frame = emptyFrame.clone();
    objectContainer.push_back(this);
}

camera::~camera(){
    argos::LOG << "Destroy camera obj" << std::endl;

    try
    {
        if (!cv::getWindowProperty(this->windowName, cv::WND_PROP_VISIBLE))
        {
            cv::destroyWindow(this->windowName);
        }
        if ( std::find(objectContainer.begin(), objectContainer.end(), this) != objectContainer.end() )
        {
            objectContainer.erase(std::remove(objectContainer.begin(), objectContainer.end(), this), objectContainer.end());
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << e.what() << "In camera object destuctor " << '\n';

    }
    

}

/**
 * Draw box into the frame
 * @param box is the box entity in argos
 * @param color is the color of the show box, default is black
*/
void camera::AddBox(argos::CBoxEntity* box, cv::Scalar color)
{
    camera::polygon polygon;
    polygon = findCPositions(box);
    cv::fillPoly(frame, polygon.corners, color);
    // for (size_t i = 0; i < polygon.corners.size(); i++)
    // {
    //     argos::LOG << "corner " << std::to_string(i) << ": " << polygon.corners[i]/(SCALE/100) << "\n";
    // }     
}

/**
 * Draw box into the frame
 * @param box is the box entity in argos
 * @param color is the color of the show box, default is black
*/

void camera::AddObject(CPrototypeEntity* object, cv::Scalar color) 
{
    camera::polygon polygon;
    polygon = findObjectCorners(object);
    cv::fillPoly(frame, polygon.corners, color);
}

/**
 * Draw robot into the frame
 * @param robot is the robot coordiante
 * @param robotRadius is the robot radius in meter 
*/
void camera::AddRobotPosition(argos::CVector3 robot, float robotRadius, cv::Scalar color, int thickness)
{
    cv::Point robot_position(robot.GetX()*SCALE, robot.GetY()*SCALE);
    //cv::circle(frame, robot_position, robotRadius * SCALE, color, thickness);
    // argos::LOG << "robot position: " << robot_position/(SCALE/100) << "\n";
}

/**
 * Plots the frame. The frame is fliped in the x axis to match argos.
 * OpenCV use left hand coordinate system while argos uses right hand coordinate system
*/
void camera::Plot() //shows if fliped for visual purpose
{
    cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(this->windowName, WINDOWSIZE, WINDOWSIZE);
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


void camera::PlotBox(argos::CBoxEntity* box, cv::Mat& outputMatrix) 
{
    outputMatrix = emptyFrame.clone();
    camera::polygon polygon;
    polygon = findCPositions(box);
    cv::fillPoly(outputMatrix, polygon.corners, cv::Scalar(0,0,0));
}

cv::Mat camera::PlotBox(argos::CBoxEntity* box) 
{
    cv::Mat outputMatrix = emptyFrame.clone();
    camera::polygon polygon;
    polygon = findCPositions(box);
    cv::fillPoly(outputMatrix, polygon.corners, cv::Scalar(0,0,0));
    return outputMatrix.clone();
}

void camera::PlotBox(argos::CPrototypeEntity* object, cv::Mat& outputMatrix) 
{
    outputMatrix = emptyFrame.clone();
    camera::polygon polygon;
    polygon = findObjectCorners(object);
    cv::fillPoly(outputMatrix, polygon.corners, cv::Scalar(0,0,0));
}

cv::Mat camera::PlotBox(argos::CPrototypeEntity* object) 
{
    cv::Mat outputMatrix = emptyFrame.clone();
    camera::polygon polygon;
    polygon = findObjectCorners(object);
    cv::fillPoly(outputMatrix, polygon.corners, cv::Scalar(0,0,0));
    return outputMatrix.clone();
}






size_t camera::windowCounter = 0;
cv::Mat camera::emptyFrame(IMAGESIZE,IMAGESIZE, CV_8UC3, cv::Scalar(255,255,255));
//cv::Mat camera::frame = camera::emptyFrame.clone();
std::vector<camera*> camera::objectContainer;
bool camera::boxExist = true;
bool camera::objectExist = true;
bool camera::footbotExist = true;
bool camera::epuckExist = true;