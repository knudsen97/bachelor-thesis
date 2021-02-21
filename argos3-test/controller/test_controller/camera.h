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

//using namespace argos;

class camera : public argos::CLoopFunctions {

public:
    struct polygon
    {
        std::vector<cv::Point2i> corners;
    };

    virtual void Init();
    virtual void PreStep();

    camera();
    camera(std::string name);
    void AddBox(argos::CBoxEntity* box);
    void AddRobotPosition(argos::CVector3 robot, int robotRadius = 15);
    void Plot();
    cv::Mat GetPlot();
    void ClearPlot();

private:
    std::vector<argos::CBoxEntity*> boxes;
    std::vector<argos::CFootBotEntity*> bots;
    std::string windowNameTemp = "Utils plot";
    std::string windowName;
    static size_t windowCounter;
    static cv::Mat emptyFrame;
    cv::Mat frame;
};

#endif