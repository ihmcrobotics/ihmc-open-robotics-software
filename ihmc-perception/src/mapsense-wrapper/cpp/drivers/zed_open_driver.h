#include <cstdio>
#include "videocapture.hpp"

#include <iostream>
#include <iomanip>

#include "videocapture.hpp"
#include "sensorcapture.hpp"

class ZEDOpenDriver
{
    public:
        ZEDOpenDriver();
        
        bool GetFrameStereoYUV(uint8_t* yuvBytes, int* dims);

        bool GetFrameDimensions(int* dims);

    private:
        int _count = 0;
        sl_oc::video::VideoParams _params;

        sl_oc::video::VideoCapture _cap;
};


