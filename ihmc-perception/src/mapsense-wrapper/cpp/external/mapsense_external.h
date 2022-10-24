#pragma once

class MapsenseExternal
{
    public:

        void extractPlanarRegionsFromPointCloud(float* points, int numPoints);
        void printMat(float* buffer, int height, int width);
        void loadMat();

    private:
        int _id = 0;


};