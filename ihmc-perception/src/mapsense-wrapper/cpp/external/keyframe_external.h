#include "stdint.h"

struct KeyframeExternal
{
    public:

        uint32_t keyframeID = 0;
        uint32_t numberOfLandmarks = 0;

        float* odometry;
        uint32_t* landmarkIDs;
};