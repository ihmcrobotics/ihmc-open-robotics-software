#pragma once

// #include "core.h"
#include "application_state.h"
#include "visual_odometry.h"

class VisualOdometryExternal
{
    public:
        VisualOdometryExternal(int nFeatures, int minFeatures);
        ~VisualOdometryExternal() {delete _visualOdometry;}

        void printMat(float* buffer, int height, int width);
        void printMat(uint8_t* buffer, int height, int width);
        
        void displayMat(uint8_t* buffer, int height, int width, int delayMilliSeconds);
        
        // void updateMonocular(uint8_t* buffer, int height, int width);
        bool updateStereo(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width,
                        double* latestPose, int* ids, double* latestPoints, int numPoints);
        

        void getExternalKeyframe(double* odometry, uint32_t* id);

        uint32_t getExternalLandmarks(float* landmarksToPack, uint32_t* idsToPack, uint32_t maxSize);

    private:
        
        int _id = 0;

        ApplicationState _appState;
        VisualOdometry* _visualOdometry;
};