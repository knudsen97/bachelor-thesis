#include "utils.h"
#include <cmath>

#define SCALE 200

utils::polygon findCPositions(argos::CBoxEntity* mBox)
{
    utils::polygon polygon;
    //Find the box' origin/center of mass:
    cv::Point2d origin = {mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), mBox->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()};
    
    //Get the Orientation of the box:
    argos::CRadians xAngle, yAngle, zAngle;
    mBox->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(xAngle, yAngle, zAngle);
    float theta = xAngle.GetValue();
    
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

    polygon.corners.push_back({c1y*SCALE, c1x*SCALE});
    polygon.corners.push_back({c2y*SCALE, c2x*SCALE});
    polygon.corners.push_back({c3y*SCALE, c3x*SCALE});
    polygon.corners.push_back({c4y*SCALE, c4x*SCALE});

    return polygon;
}


utils::utils()
{
    std::string name;
    name.append(utils::window_name_temp);
    name.append(" ");
    name.append(std::to_string(utils::window_counter));
    cv::namedWindow(name);
    utils::window_name = name;
    utils::window_counter++;
    frame = empty_frame.clone();
}

utils::utils(std::string name)
{
    cv::namedWindow(name);
    utils::window_name = name;
    utils::window_counter++;
    frame = empty_frame.clone();
}

void utils::plot(argos::CBoxEntity* box)
{
    utils::polygon polygon;
    polygon = findCPositions(box);
    cv::polylines(frame, polygon.corners, true, cv::Scalar(0,0,0), 2);
    //cv::flip(frame, frame, 1);
    cv::imshow(this->window_name, frame);
    for (size_t i = 0; i < polygon.corners.size(); i++)
    {
        argos::LOG << "corner " << std::to_string(i) << ": " << polygon.corners[i]/SCALE << "\n";
    }     
}

void utils::plot(argos::CVector3 robot)
{
    cv::Point robot_position(robot.GetX()*SCALE, robot.GetY()*SCALE);
    cv::circle(frame, robot_position, 2, cv::Scalar(255, 255, 000));
    //cv::flip(frame, frame, 1);
    cv::imshow(this->window_name, frame);
    argos::LOG << "robot position: " << robot_position/SCALE << "\n";
}

void utils::clear_plot()
{
    utils::frame = utils::empty_frame.clone();
}




size_t utils::window_counter = 0;
cv::Mat utils::empty_frame(500,500, CV_8UC3, cv::Scalar(255,255,255));