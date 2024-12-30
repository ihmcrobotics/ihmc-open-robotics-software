// http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf

#define COLOR_RANGE 256
#define MAX_COLOR_VAL 255
#define PERIOD 512

#include "Utils.cu"

extern "C"
__global__
void colorizeDepth(unsigned short* depthImage, size_t depthPitch,
                   unsigned char* colorizedImage, size_t colorizedPitch,
                   int rows, int cols)
{
    // Find the X index and stride of this thread
    int coordX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    // Find the Y index and stride of this thread
    int coordY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    // Use a grid-stride loop
    for (int y = coordY; y < rows; y += strideY)
    {
        unsigned short* depthRow = (unsigned short*)((char*) depthImage + y * depthPitch);
        unsigned char* colorizedRow = (unsigned char*) ((char*) colorizedImage + y * colorizedPitch);

        for (int x = coordX; x < cols; x += strideX)
        {
            unsigned short depthValue = depthRow[x];

            unsigned char normalizedDepth = 255 - (depthValue + 128) / COLOR_RANGE;
            
            unsigned short ha = depthValue % PERIOD;
            if (ha > MAX_COLOR_VAL)
                ha = PERIOD - ha - 1;

            unsigned short hb = (depthValue + 3 * PERIOD / 4) % PERIOD;
            if (hb > MAX_COLOR_VAL)
                hb = PERIOD - hb - 1;

            colorizedRow[3 * x + 0] = normalizedDepth;  // B
            colorizedRow[3 * x + 1] = ha;               // G
            colorizedRow[3 * x + 2] = hb;               // R
        }
    }
}

extern "C"
__global__
void deColorizeDepth(unsigned char* colorizedImage, size_t colorizedPitch,
                     unsigned short* depthImage, size_t depthPitch,
                     int rows, int cols)
{
    // Find the X coordinate and stride of this thread
    int coordX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    // Find the Y coordinate and stride of this thread
    int coordY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    for (int y = coordY; y < rows; y += strideY)
    {
        unsigned char* colorizedRow = (unsigned char*) ((char*) colorizedImage + y * colorizedPitch);
        unsigned short* depthRow = (unsigned short*)((char*) depthImage + y * depthPitch);

        for (int x = coordX; x < cols; x += strideX)
        {
            unsigned char normalizedDepth = 255 - colorizedRow[3 * x + 0];
            unsigned char ha = colorizedRow[3 * x + 1];
            unsigned char hb = colorizedRow[3 * x + 2];

            unsigned int depthLevel = COLOR_RANGE * normalizedDepth;

            char mL = (4 * depthLevel / PERIOD - 1) % 4;
            if (mL < 0)
                mL = 4 + mL;

            short moddedValueC = (depthLevel - PERIOD / 8) % PERIOD;
            if (moddedValueC < 0)
                moddedValueC = PERIOD - moddedValueC;
            unsigned short preciseDepthLevel = depthLevel - moddedValueC + mL * PERIOD / 4 - PERIOD / 8;

            unsigned short delta;
            if (mL == 0)
                delta = ha;
            else if (mL == 1)
                delta = hb;
            else if (mL == 2)
                delta = MAX_COLOR_VAL - ha;
            else if (mL == 3)
                delta = MAX_COLOR_VAL - hb;

            depthRow[x] = preciseDepthLevel + delta;
        }
    }
}