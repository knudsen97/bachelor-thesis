#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>


#include <string>
#include <vector>

//using namespace argos;

class utils
{

public:
    struct polygon
    {
        std::vector<cv::Point2i> corners;
    };

    utils();
    utils(std::string name);
    void addBox(argos::CBoxEntity* box);
    void addRobotPosition(argos::CVector3 robot, int robotRadius = 15);
    void plot();
    cv::Mat getPlot();
    void clear_plot();

private:
    std::string window_name_temp = "Utils plot";
    std::string window_name;
    static size_t window_counter;
    static cv::Mat empty_frame;
    cv::Mat frame;
};