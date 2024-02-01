#include "zed_open_driver.h"

class ZEDOpenDriverExternal
{
   public:
      ZEDOpenDriverExternal(int resolution, int fps) : zed(resolution, fps) {}

      bool getFrameStereoYUVExternal(uint8_t* yuvBytes, int* dims);

      bool getFrameDimensions(int* dims);

   private:
      ZEDOpenDriver zed;
};