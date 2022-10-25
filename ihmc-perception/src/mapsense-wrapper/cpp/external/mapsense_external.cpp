#include "mapsense_external.h"
#include "iostream"

MapsenseExternal::MapsenseExternal()
{
    _openCL = new OpenCLManager("/home/quantum/Workspace/Code/IHMC/repository-group/ihmc-open-robotics-software/ihmc-perception/src/mapsense-wrapper/cpp");

   _regionCalculator = new PlanarRegionCalculator(appState);
   _regionCalculator->setOpenCLManager(_openCL);

   std::cout << "MapsenseExternal Created" << std::endl;
}

void MapsenseExternal::printMat(float* buffer, int height, int width)
{
    std::cout << "Height: " << height << " Width: " << width << std::endl;
}


void MapsenseExternal::loadMat()
{
    // cv::Mat mat = cv::imread("/home/quantum/Pictures/Profiler_2.jpg");

    // std::cout << "Height: " << mat.rows << std::endl;

    // cv::imshow("Window", mat);
    // cv::waitKey(0);
}

void MapsenseExternal::extractPlanarRegionsFromPointCloud(float* points, int numPoints)
{
    for(int i = 0; i<numPoints; i++)
    {
        std::cout << "Point: " << points[i*3] << ", " << points[i*3+1] << ", " << points[i*3+2] << std::endl;
    }

}