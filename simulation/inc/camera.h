#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <string>
#include <vector>
#include <cmath>


#define IMAGESIZE 700
#define WINDOWSIZE 500
#define ARENASIZE 3
#define SCALE (IMAGESIZE/ARENASIZE)

class camera : public argos::CLoopFunctions
{
public:
    struct polygon
    {
        std::vector<cv::Point2i> corners;
    };

    void step();

    camera();
    camera(std::string name);
    ~camera();

    /**
     * Draw box into the frame
     * @param box is the box entity in argos
     * @param color is the color of the show box, default is black
    */
    void AddBox(argos::CBoxEntity* box, cv::Scalar color = {0,0,0});

    /**
     * Draw robot into the frame
     * @param robot is the robot coordiante
     * @param robotRadius is the robot radius in meter 
     * @param color color of the plottet robot
     * @param thickness line thickness -1 for fill 
    */
    void AddRobotPosition(argos::CVector3 robot, float robotRadius = 0.085, cv::Scalar color = {255, 255, 000}, int thickness = 1);

    /**
     * Plots the frame. The frame is fliped in the x axis to match argos.
     * OpenCV use left hand coordinate system while argos uses right hand coordinate system
    */
    void Plot();

    /**
     * Gets the frame either by returning it or giving the function a output matrix.
    */
    cv::Mat GetPlot();
    void GetPlot(cv::Mat& outputFrame);
    void ClearPlot();

protected:
    std::vector<argos::CBoxEntity*> boxes;
    std::vector<argos::CFootBotEntity*> bots;
    std::string windowNameTemp = "Utils plot";
    std::string windowName;
    static size_t windowCounter;
    static cv::Mat emptyFrame;
    static cv::Mat frame;
    static std::vector<camera*> objectContainer;
};

#endif
