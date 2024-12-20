// http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf

#define MAX_DEPTH_VAL 65536.0
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

            // TODO: Figure out how to do this using only integers
            unsigned int normalizedDepth = round(MAX_COLOR_VAL * ((depthValue + 0.5) / MAX_DEPTH_VAL));
            
            unsigned int ha = depthValue % PERIOD;
            if (ha > MAX_COLOR_VAL_INT)
                ha = PERIOD - ha - 1;

            unsigned int hb = ((int) depthValue + THREE_FOURTHS_PERIOD) % PERIOD;
            if (hb > MAX_COLOR_VAL_INT)
                hb = PERIOD - hb - 1;

            colorizedRow[3 * x + 0] = normalizedDepth;   // B
            colorizedRow[3 * x + 1] = ha;      // G
            colorizedRow[3 * x + 2] = hb;      // R
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
            double normalizedDepth = colorizedRow[3 * x + 0] / MAX_COLOR_VAL;
            double ha = colorizedRow[3 * x + 1] / MAX_COLOR_VAL;
            double hb = colorizedRow[3 * x + 2] / MAX_COLOR_VAL;

            int mL = ((int) (4.0 * normalizedDepth / NORMALIZED_PERIOD - 0.5)) % 4;
            if (mL < 0)
                mL = 4 + mL;

            double moddedValue = fmod(normalizedDepth - 0.125 * NORMALIZED_PERIOD, NORMALIZED_PERIOD);
            if (moddedValue < 0.0)
                moddedValue = NORMALIZED_PERIOD + moddedValue;

            double depthLevel = normalizedDepth - moddedValue + (0.25 * NORMALIZED_PERIOD * mL) - 0.125 * NORMALIZED_PERIOD;

            double delta;
            if (mL == 0)
                delta = 0.5 * NORMALIZED_PERIOD * ha;
            else if (mL == 1)
                delta = 0.5 * NORMALIZED_PERIOD * hb;
            else if (mL == 2)
                delta = 0.5 * NORMALIZED_PERIOD * (1.0 - ha);
            else if (mL == 3)
                delta = 0.5 * NORMALIZED_PERIOD * (1.0 - hb);

            unsigned short depthValue = (unsigned short) round(MAX_DEPTH_VAL * (depthLevel + delta));
            depthRow[x] = depthValue;
        }
    }
}