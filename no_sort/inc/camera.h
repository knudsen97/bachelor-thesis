#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <vector>
#include <cmath>
#include <iterator>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_link_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_link_entity.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>
#include <argos3/plugins/robots/prototype/simulator/dynamics3d_prototype_model.h>







#define IMAGESIZE 700
#define WINDOWSIZE 500
#define ARENASIZE 6
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
     * Draw arbitrary object into the frame
     * @param object is the box entity in argos
     * @param color is the color of the show box, default is black
    */
    void AddObject(argos::CPrototypeEntity* object, cv::Scalar color = {0,0,0});

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
    void PlotBox(argos::CBoxEntity* box, cv::Mat& outputMatrix);
    cv::Mat PlotBox(argos::CBoxEntity* box);
    void PlotBox(argos::CPrototypeEntity* object, cv::Mat& outputMatrix);
    cv::Mat PlotBox(argos::CPrototypeEntity* object);

protected:
    static bool boxExist;
    static bool objectExist;
    static bool footbotExist;
    static bool epuckExist;
    std::vector<argos::CBoxEntity*> boxes;
    std::vector<argos::CEPuckEntity*> bots;
    std::string windowNameTemp = "Utils plot";
    std::string windowName;
    static size_t windowCounter;
    static cv::Mat emptyFrame;
    cv::Mat frame;
    static std::vector<camera*> objectContainer;
};

#endif
