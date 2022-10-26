#include "mapsense_external.h"
#include "iostream"

MapsenseExternal::MapsenseExternal()
{
    // _openCL = new OpenCLManager("/home/quantum/Workspace/Code/IHMC/repository-group/ihmc-open-robotics-software/ihmc-perception/src/mapsense-wrapper/cpp");
    _openCL = new OpenCLManager("/home/bmishra/Workspace/Code/repository-group/ihmc-open-robotics-software/ihmc-perception/src/mapsense-wrapper/cpp");

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
    cv::Mat mat = cv::imread("/home/bmishra/Pictures/20221011_BhavyanshMishra02.jpg");

    std::cout << "Height: " << mat.rows << std::endl;

    cv::imshow("Window", mat);
    cv::waitKey(0);
}

void MapsenseExternal::extractPlanarRegionsFromPointCloud(float* points, int numPoints)
{

    auto start_point = std::chrono::steady_clock::now();

    std::vector<float> vertices;
    
    for(int i = 0; i<numPoints; i++)
    {
        Eigen::Vector3f point(points[i*3], points[i*3+1], points[i*3+2]);

        if(point.norm() > 0.1f)
        {
            vertices.push_back(point.x());
            vertices.push_back(point.y());
            vertices.push_back(point.z());
        }

        // std::cout << "Point: " << points[i*3] << ", " << points[i*3+1] << ", " << points[i*3+2] << std::endl;
    }

    // Call planar region calculator method to load pointcloud and return planar regions.
    _regionCalculator->GenerateRegionsFromPointCloud(appState, vertices);


    printf("Total Points: %d\n", (int)(vertices.size()/3));

    auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   printf("Total Time to Generate Regions: %.3lf ms\n", duration);

}

void MapsenseExternal::testOpenCLParallelAdd(float* bufferA, float* bufferB, float* bufferOutput, int numFloats)
{
    uint8_t idBufferA = _openCL->CreateLoadBufferFloat(bufferA, numFloats);
    uint8_t idBufferB = _openCL->CreateLoadBufferFloat(bufferB, numFloats);
    uint8_t idBufferOutput = _openCL->CreateBufferFloat(numFloats);

    _openCL->SetArgument("parallelAddKernel", 0, idBufferA);
    _openCL->SetArgument("parallelAddKernel", 1, idBufferB);
    _openCL->SetArgument("parallelAddKernel", 2, idBufferOutput);

    _openCL->commandQueue.enqueueNDRangeKernel(_openCL->parallelAddKernel, cl::NullRange, cl::NDRange(numFloats), cl::NullRange);

    _openCL->ReadBufferFloat(idBufferOutput, bufferOutput, numFloats);

    printf("Parallel Add Completed\n");

}