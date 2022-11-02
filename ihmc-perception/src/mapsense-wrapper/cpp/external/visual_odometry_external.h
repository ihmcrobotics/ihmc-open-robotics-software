#pragma once

// #include "core.h"
#include "application_state.h"
#include "visual_odometry.h"

class VisualOdometryExternal
{
    public:
        VisualOdometryExternal();
        ~VisualOdometryExternal() {delete _visualOdometry;}

        void printMat(float* buffer, int height, int width);
        void printMat(uint8_t* buffer, int height, int width);
        
        void displayMat(uint8_t* buffer, int height, int width, int delayMilliSeconds);
        
        // void updateMonocular(uint8_t* buffer, int height, int width);
        void updateStereo(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width);
        

    private:
        
        int _id = 0;

        ApplicationState _appState;
        VisualOdometry* _visualOdometry;
};