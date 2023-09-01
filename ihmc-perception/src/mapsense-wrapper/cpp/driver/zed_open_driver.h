#include <cstdio>
#include "videocapture.hpp"

#include <iostream>
#include <iomanip>

#include "videocapture.hpp"
#include "sensorcapture.hpp"

struct ZEDParams
{
   public: 
      ZEDParams(int resolution, int fps)
      {
         switch(resolution)
         {
               case 720: {params.res = sl_oc::video::RESOLUTION::HD720;break;}
               case 1080: {params.res = sl_oc::video::RESOLUTION::HD1080;break;}
               case 2160: {params.res = sl_oc::video::RESOLUTION::HD2K;break;}
               default: {params.res = sl_oc::video::RESOLUTION::HD1080;break;}
         }

         switch(fps)
         {
               case 15: {params.fps = sl_oc::video::FPS::FPS_15;break;}
               case 30: {params.fps = sl_oc::video::FPS::FPS_30;break;}
               case 60: {params.fps = sl_oc::video::FPS::FPS_60;break;}
               case 100: {params.fps = sl_oc::video::FPS::FPS_100;break;}
               default: {params.fps = sl_oc::video::FPS::FPS_30;break;}
         }

         
      }

      sl_oc::video::VideoParams params;
};


class ZEDOpenDriver
{
   public:
      ZEDOpenDriver(int resolution, int fps);
      
      bool GetFrameStereoYUV(uint8_t* yuvBytes, int* dims);

      bool GetFrameDimensions(int* dims);

   private:
      int _count = 0;


      ZEDParams _params;

      sl_oc::video::VideoCapture _cap;
};


