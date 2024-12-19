#define MAX_DEPTH_VAL 65535.0
#define MAX_COLOR_VAL 255.0
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
            double normalizedDepth = depthValue / MAX_DEPTH_VAL;

            double ha = fmod(normalizedDepth / (0.5 * NORMALIZED_PERIOD), 2.0);
            if (ha > 1.0)
                ha = 2.0 - ha;

            // TODO: This is ha +/- 0.5
            double hb = fmod((normalizedDepth - 0.25 * NORMALIZED_PERIOD) / (0.5 * NORMALIZED_PERIOD), 2.0);
            if (hb < 0.0)
                hb = 2.0 + hb;
            if (hb > 1.0)
                hb = 2.0 - hb;

            unsigned char colorDepth = (unsigned char) round(MAX_COLOR_VAL * normalizedDepth);
            unsigned char colorHa = (unsigned char) round(MAX_COLOR_VAL * ha);
            unsigned char colorHb = (unsigned char) round(MAX_COLOR_VAL * hb);

            colorizedRow[3 * x + 0] = colorDepth;   // B
            colorizedRow[3 * x + 1] = colorHa;      // G
            colorizedRow[3 * x + 2] = colorHb;      // R
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