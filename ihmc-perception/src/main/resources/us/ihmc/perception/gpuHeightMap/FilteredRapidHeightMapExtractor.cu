#define LAYERS 6  // Set a fixed size

extern "C"
__global__
void filterRapidHeightMap(unsigned short * matrixPointer, size_t pitchA,
                         unsigned short * resultPointer, size_t pitchResult,
                         unsigned short * newestHeightMap, size_t pitchNewestHeightMap,
                         size_t layerSize, int rows, int cols, int defaultValue)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    if (indexX >= cols || indexY >= rows)
        return;

    int sum = 0;
    float variance[LAYERS] = {0};

    // Compute the average height of the history in order to get the variance at each layer
    for (int layer = 0; layer < LAYERS; layer++)
    {
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);
        unsigned short * matrixPointerRow = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

//         printf("Layer: %d, Value: %d, %d, and %d\n", layer, indexY, indexX, (int) *matrixPointerRow);
        sum += (int) *matrixPointerRow;
    }

    unsigned short avg = sum / LAYERS;

    // Compute the variance for each layer
    for (int layer = 0; layer < LAYERS; layer++)
    {
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);
        unsigned short * matrixPointerRow = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

        float diff = (float)(*matrixPointerRow) - avg;
        variance[layer] = abs(diff);
    }

    double heightSum = 0;
    double varianceSum = 0;

    // Compute the height and variance sum
    for (int layer = 0; layer < LAYERS; layer++)
    {
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);
        unsigned short * matrixPointerRow = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

        if (variance[layer] > 0.0f)  // Prevent division by zero
        {
            heightSum += (double)(*matrixPointerRow) / (double)variance[layer];
            varianceSum += 1.0 / (double)variance[layer];
        }
    }

//     printf("Height sum: %f\n", heightSum);
//     printf("Variance sum: %f\n", varianceSum);
    unsigned short newHeight = heightSum / varianceSum;

    unsigned short * heightValue = (unsigned short*)((char*) newestHeightMap + indexY * pitchNewestHeightMap) + indexX;

    int heightValueInt = (int) *heightValue;
    float diff = (float) heightValueInt - avg;
    float newVariance = abs(diff);

    if (*heightValue == defaultValue)
        return;


    float alpha = 0.2;
//     printf("Equation parameters (heightValue %hu) (alpha %f) (avg %hu)\n", *heightValue, alpha, avg);
    float heightEstimate = (float) *heightValue * alpha + (avg * (1.0 - alpha)); // (newHeight * newVariance) / newVariance;

    unsigned short *outputPointer = (unsigned short *)((char*) resultPointer + indexY * pitchResult) + indexX;
    *outputPointer = (unsigned short) heightEstimate;

//     printf("GPU Height Estimate: %f\n", heightEstimate);
}

