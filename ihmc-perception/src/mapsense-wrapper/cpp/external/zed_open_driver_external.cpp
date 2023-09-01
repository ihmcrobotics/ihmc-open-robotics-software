#include "zed_open_driver_external.h"

bool ZEDOpenDriverExternal::getFrameStereoYUVExternal(uint8_t* yuvBytes, int* dims)
{
   return zed.GetFrameStereoYUV(yuvBytes, dims);
}

bool ZEDOpenDriverExternal::getFrameDimensions(int* dims)
{
   return zed.GetFrameDimensions(dims);
}