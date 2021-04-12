#ifndef PLANNER_H
#define PLANNER_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include "queue"
#include "algorithm"
#include "../inc/camera.h"

#define MAX_USHORT 65535
#define OFF_SET 0.3

class planner
{
public:
    struct cPositions{
        argos::CVector3 c1, c2, c3, c4;
    };

    planner();
    ~planner();
    
    cPositions FindCPositions(argos::CBoxEntity* mBox);
    std::vector<argos::CVector3> FindPushPoints(argos::CBoxEntity* mBox, argos::CVector3 goalPoint);

    //cv::Mat Wavefront(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal);
    //Debug function: 
    cv::Mat Wavefront(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal);

    //std::vector<cv::Point> Pathfinder(cv::Mat &grayMap, argos::CVector3 &robot, argos::CVector3 &goal);
    //Debug function: 
    std::vector<cv::Point> Pathfinder(cv::Mat &grayMap, argos::CVector3 &robot, argos::CVector3 &goal);

    /*Not used right now*/
    std::vector<cv::Point> PostProcessing(std::vector<cv::Point> &subGoals);
    
    /*Auxiliary functions*/
    template<class V>
    argos::Real Projection(V &v1, V &v2);
    argos::CVector3 push(argos::CBoxEntity* mBox, argos::CVector3 currentPoint, argos::CVector3 goalPoint);
    argos::CVector3 translate(argos::CVector3 point, argos::CRadians orientation, argos::Real distance);
    bool ValidLine(cv::Point A, cv::Point B, cv::Mat img);
    ushort GrayPixelVal(cv::Mat &map, cv::Point point);

    

private:
    cv::Mat map; //For illustration
    cv::Mat grayMapCopy;
};

#endif