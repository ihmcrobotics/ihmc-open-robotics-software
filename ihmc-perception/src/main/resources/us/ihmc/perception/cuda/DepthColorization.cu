/*
 * Functions for "colorizing" and "de-colorizing" depth data.
 * In other words, these functions convert depth images between
 * 16 bit 1 channel (normal depth) and 8 bit 3 channel (colorized depth) images.
 *
 * To colorize the depth D into YUV channels, the following formulas are used:
 *  - Y = floor(sqrt(D))
 *  - U = floor((D - Y^2) / 2)
 *  - V = ceil((D - Y^2) / 2)
 *
 * To de-colorize the depth, the following formula is used:
 *  - D = Y^2 + U + V
 *
 * Semi-intuitively, Y represents the rough depth value. It is a quantization of D into the range [0, 255].
 * A square root is preferred over a linear mapping (e.g. Y = D / 256) as the resulting quantization
 * is finer (preserves more detail) at closer ranges, where depth sensors are naturally more accurate.
 * The quantization does, however, become coarser at longer ranges (losing detail) compared to a linear mapping.
 *
 * The U and V components combine to represent the detail lost from the quantization.
 * The quantized depth contains gaps in values, increasing in size as the depth value grows.
 * The maximum size of such a gap is 510, identical to the maximum value of U + V.
 * Thus, U + V is used as the detail to fill in the gaps.
 */

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

    // Declare variables
    unsigned short* depthRow;
    unsigned char* colorizedRow;

    unsigned short depthValue;  // D

    unsigned char sqrtDepth;    // Y
    unsigned short adjustment;  // U + V
    unsigned char adjustmentA;  // U
    unsigned char adjustmentB;  // V

    // Use a grid-stride loop
    for (int y = coordY; y < rows; y += strideY)
    {
        depthRow = (unsigned short*)((char*) depthImage + y * depthPitch);
        colorizedRow = (unsigned char*)((char*) colorizedImage + y * colorizedPitch);

        for (int x = coordX; x < cols; x += strideX)
        {
            depthValue = depthRow[x];

            sqrtDepth = sqrtf((float) depthValue);
            adjustment = depthValue - sqrtDepth * sqrtDepth;

            adjustmentA = adjustmentB = adjustment / 2;
            adjustmentB += adjustment % 2;

            colorizedRow[3 * x + 0] = sqrtDepth;    // Y
            colorizedRow[3 * x + 1] = adjustmentA;  // U
            colorizedRow[3 * x + 2] = adjustmentB;  // V
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

    // Declare variables
    unsigned char* colorizedRow;
    unsigned short* depthRow;

    unsigned char sqrtDepth;    // Y
    unsigned char adjustmentA;  // U
    unsigned char adjustmentB;  // V

    unsigned short depthValue;  // D

    // Use a grid-stride loop
    for (int y = coordY; y < rows; y += strideY)
    {
        colorizedRow = (unsigned char*)((char*) colorizedImage + y * colorizedPitch);
        depthRow = (unsigned short*)((char*) depthImage + y * depthPitch);

        for (int x = coordX; x < cols; x += strideX)
        {
            sqrtDepth = colorizedRow[3 * x + 0];
            adjustmentA = colorizedRow[3 * x + 1];
            adjustmentB = colorizedRow[3 * x + 2];

            depthValue = sqrtDepth * sqrtDepth + adjustmentA + adjustmentB;

            depthRow[x] = depthValue;
        }
    }
}