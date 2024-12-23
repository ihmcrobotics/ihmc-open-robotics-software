// http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf

#define MAX_DEPTH_VAL 65536.0
#define MAX_DEPTH_VAL_INT 65536
#define MAX_COLOR_VAL 255.0
#define MAX_COLOR_VAL_INT 255
#define PERIOD 512
#define THREE_FOURTHS_PERIOD 384
#define NORMALIZED_PERIOD 0.0078125

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

            unsigned char normalizedDepth = (depthValue + 128) / 256;
            
            unsigned short ha = depthValue % PERIOD;
            if (ha > MAX_COLOR_VAL_INT)
                ha = PERIOD - ha - 1;

            unsigned short hb = (depthValue + THREE_FOURTHS_PERIOD) % PERIOD;
            if (hb > MAX_COLOR_VAL_INT)
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
            unsigned char normalizedDepth = colorizedRow[3 * x + 0];
            unsigned char ha = colorizedRow[3 * x + 1];
            unsigned char hb = colorizedRow[3 * x + 2];

            char mL = ((4 * 256 * normalizedDepth) / PERIOD - 1) % 4;
            if (mL < 0)
                mL = 4 + mL;

            unsigned int depthLevel = 256 * normalizedDepth;
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
                delta = 255 - ha;
            else if (mL == 3)
                delta = 255 - hb;

            depthRow[x] = preciseDepthLevel + delta;
        }
    }
}