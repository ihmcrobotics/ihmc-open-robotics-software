#include "visual_odometry_external.h"
#include "iostream"

VisualOdometryExternal::VisualOdometryExternal()
{
   std::cout << "VisualOdometryExternal Created" << std::endl;

   _visualOdometry = new VisualOdometry(_appState);
}

void VisualOdometryExternal::printMat(float* buffer, int height, int width)
{
    std::cout << "Height: " << height << " Width: " << width << std::endl;

    for(int i = 0; i<height; i++)
    {
        for(int j = 0; j<width; j++)
        {
            printf("%.2lf\t", buffer[i*width + j]);
        }
        printf("\n");
    }
}

void VisualOdometryExternal::printMat(uint8_t* buffer, int height, int width)
{
    std::cout << "[IntBuffer] Height: " << height << " Width: " << width << std::endl;

    

    for(int i = 0; i<height; i++)
    {
        for(int j = 0; j<width; j++)
        {
            printf("%d\t", buffer[i*width + j]);
        }
        printf("\n");
    }
}

void VisualOdometryExternal::displayMat(uint8_t* buffer, int height, int width, int delayMilliSeconds)
{
    cv::Mat mat(height, width, CV_8UC3, buffer);
    cv::imshow("Image", mat);
    cv::waitKey(delayMilliSeconds);
}

void VisualOdometryExternal::updateMonocular(uint8_t* buffer, int height, int width)
{
    _visualOdometry->UpdateMonocular(cv::Mat(height, width, CV_8UC3, buffer));
}

void VisualOdometryExternal::updateStereo(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
{
    _visualOdometry->UpdateStereo(cv::Mat(height, width, CV_8UC3, bufferLeft), cv::Mat(height, width, CV_8UC3, bufferRight));
}
