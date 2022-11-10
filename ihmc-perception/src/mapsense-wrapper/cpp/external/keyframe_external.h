#include "stdint.h"

struct KeyframeExternal
{
    public:

        uint32_t keyframeID = 0;

        float odometry[16];
};