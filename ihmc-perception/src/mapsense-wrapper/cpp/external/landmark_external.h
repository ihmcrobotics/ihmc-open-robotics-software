#include "stdint.h"

struct LandmarkExternal
{
    public:

        uint32_t landmarkID = 0;
        uint32_t numberOfMeasurements = 0;

        float* measurement;
        float* point3d;

        uint32_t* keyframeIDs;
};