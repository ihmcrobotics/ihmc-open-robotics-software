#include "zed_open_driver.h"


ZEDOpenDriver::ZEDOpenDriver(int resolution, int fps) : _params(resolution, fps), _cap({_params.params})
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
      std::copy(  frame.data, frame.data + dims[0] * dims[1] * dims[2], yuvBytes);
      
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
      dims[2] = 2;

      return true;
   }

   return false;
}
