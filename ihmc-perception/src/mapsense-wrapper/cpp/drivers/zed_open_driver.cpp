#include "zed_open_driver.h"


ZEDOpenDriver::ZEDOpenDriver() : _cap(_params)
{
    if (!_cap.initializeVideo())
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;
    }
    std::cout << "Connected to camera sn: " << _cap.getSerialNumber() << std::endl;
}

bool ZEDOpenDriver::GetFrameStereoYUV(uint8_t* yuvBytes, int* dims)
{
    const sl_oc::video::Frame frame = _cap.getLastFrame();
   
    if (frame.data != nullptr)
    {
        dims[0] = frame.height;
        dims[1] = frame.width;

        std::copy(  frame.data, frame.data + frame.height * frame.width, yuvBytes);
        
        return true;
    }

    return false;
}

bool ZEDOpenDriver::GetFrameDimensions(int* dims)
{
    const sl_oc::video::Frame frame = _cap.getLastFrame();
   
    if (frame.data != nullptr)
    {
        dims[0] = frame.height;
        dims[1] = frame.width;

        return true;
    }

    return false;
}
