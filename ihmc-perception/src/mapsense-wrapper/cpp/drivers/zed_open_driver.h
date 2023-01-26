#include <cstdio>
#include "videocapture.hpp"

#include <iostream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ZEDOpenDriver
{
    public:
        ZEDOpenDriver();
        void Update(bool display);

    private:
        int _count = 0;
        sl_oc::video::VideoParams _params(sl_oc::video::RESOLUTION::HD720, sl_oc::video::FPS::FPS_60, sl_oc::VERBOSITY::ERROR);

        sl_oc::video::VideoCapture _cap;
};


