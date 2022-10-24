#include "mapsense_external.h"
#include "iostream"
#include "opencv2/highgui.hpp"

void MapsenseExternal::printMat(float* buffer, int height, int width)
{
    std::cout << "Height: " << height << " Width: " << width << std::endl;
}

void MapsenseExternal::loadMat()
{
    cv::Mat mat = cv::imread("/home/quantum/Pictures/Profiler_2.jpg");

    std::cout << "Height: " << mat.rows << std::endl;

    cv::imshow("Window", mat);
    cv::waitKey(0);
}