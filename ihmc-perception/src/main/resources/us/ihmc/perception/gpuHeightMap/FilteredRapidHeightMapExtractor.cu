extern "C"

__global__
void filterRapidHeightMap(unsigned short * matrixPointer, size_t pitchA,
                         unsigned short * resultPointer, size_t pitchResult,
                         unsigned short * newestHeightMap, size_t pitchNewestHeightMap,
                         int layers, size_t layerSize, int rows, int cols, float alpha)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    if (indexX >= cols || indexY >= rows)
        return;

    unsigned int sum = 0;

    // Compute the average height of the history in order to get the variance at each layer
    for (int layer = 0; layer < layers; layer++)
    {
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);
        unsigned short * matrixCell = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

//         printf("Layer: %d, Value: %d, %d, and %d\n", layer, indexY, indexX, (int) *matrixCell);
        sum += (int) *matrixCell;
    }

    unsigned short avg = sum / layers;

    unsigned short * heightValue = (unsigned short*)((char*) newestHeightMap + indexY * pitchNewestHeightMap) + indexX;

    int heightValueInt = (int) *heightValue;
    float diff = (float) heightValueInt - avg;
    float newVariance = abs(diff);

//     printf("Equation parameters (heightValue %hu) (alpha %f) (avg %hu)\n", *heightValue, alpha, avg);
    float heightEstimate = (float) *heightValue * alpha + (avg * (1.0 - alpha)); // (newHeight * newVariance) / newVariance;

    unsigned short *outputPointer = (unsigned short *)((char*) resultPointer + indexY * pitchResult) + indexX;
    *outputPointer = (unsigned short) heightEstimate;
}
