#include "stdint.h"

struct LandmarkExternal
{
    public:

        uint32_t landmarkID = 0;

        float measurement[2];
        float point3d[3];
};