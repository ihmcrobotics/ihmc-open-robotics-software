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
    }

}
