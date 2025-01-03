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

            unsigned char normalizedDepth = depthValue / COLOR_RANGE;
            
            unsigned short adjustmentA = depthValue % PERIOD;
            if (adjustmentA > MAX_COLOR_VAL)
                adjustmentA = PERIOD - adjustmentA - 1;

            unsigned short adjustmentB = (depthValue + PERIOD / 2) % PERIOD;
            if (adjustmentB > MAX_COLOR_VAL)
                adjustmentB = PERIOD - adjustmentB - 1;

            colorizedRow[3 * x + 0] = normalizedDepth;  // Y
            colorizedRow[3 * x + 1] = adjustmentA;      // U
            colorizedRow[3 * x + 2] = adjustmentB;      // V
        }
    }
}

extern "C"
__global__
void deColorizeDepth(unsigned char* colorizedImage, size_t colorizedPitch,
                     unsigned short* depthImage, size_t depthPitch,
                     int rows, int cols, int noiseThreshold)
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
            unsigned char adjustmentA = colorizedRow[3 * x + 1];
            unsigned char adjustmentB = colorizedRow[3 * x + 2];

            unsigned char phase = normalizedDepth % 2;
            short deltaA;
            short deltaB;
            if (phase == 0)
            {
                deltaA = adjustmentA;
                deltaB = COLOR_RANGE - adjustmentB - 1;
            }
            else if (phase == 1)
            {
                deltaA = COLOR_RANGE - adjustmentA - 1;
                deltaB = adjustmentB;
            }

            if (abs(deltaA - deltaB) > noiseThreshold)
                depthRow[x] = 0;
            else
                depthRow[x] = COLOR_RANGE * normalizedDepth + (deltaA + deltaB) / 2;
        }
    }
}