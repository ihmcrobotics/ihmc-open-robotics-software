#pragma once

// #include "core.h"
#include "opencl_manager.h"
#include "planar_region_calculator.h"

class MapsenseExternal
{
    public:
        MapsenseExternal();
        ~MapsenseExternal() { delete _openCL; delete _regionCalculator; }

        void extractPlanarRegionsFromPointCloud(float* points, int numPoints);
        void printMat(float* buffer, int height, int width);
        void loadMat();

    private:
        
        int _id = 0;

        ApplicationState appState;
        OpenCLManager* _openCL;
        PlanarRegionCalculator* _regionCalculator;


};