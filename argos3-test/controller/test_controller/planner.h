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
#include "camera.h"

class planner
{
public:
    struct cPositions{
        argos::CVector3 c1, c2, c3, c4;
    };

    planner();
    ~planner();
    cPositions FindCPositions(argos::CBoxEntity* mBox);

    template<class V>
    argos::Real Projection(V &v1, V &v2);

    std::vector<argos::CVector3> FindPushPoints(argos::CBoxEntity* mBox, argos::CVector3 goalPoint);

    cv::Mat Wavefront(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal);
    std::vector<cv::Point> Pathfinder(cv::Mat &map, argos::CVector3 &robot, argos::CVector3 &goal);

private:
    cv::Mat map; //For illustration
};

#endif