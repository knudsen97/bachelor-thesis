#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "utils.h"

class planner : public utils
{
public:
    planner();
    ~planner();

    void wavefront(cv::Mat &map);
private:
    int test; 
};