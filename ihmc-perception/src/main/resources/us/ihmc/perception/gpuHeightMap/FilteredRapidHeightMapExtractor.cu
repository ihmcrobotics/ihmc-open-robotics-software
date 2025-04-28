extern "C"

__global__
void filterRapidHeightMap(unsigned short * matrixPointer, size_t pitchA,
                         unsigned short * resultPointer, size_t pitchResult,
                         unsigned short * newestHeightMap, size_t pitchNewestHeightMap,
                         int layers, int currentLayerIndex, size_t layerSize, int rows, int cols, float alpha, int resetOffset)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    if (indexX >= cols || indexY >= rows)
        return;

    unsigned int sum = 0;
    bool dontUseAlphaYet = false;

    // Compute the average height of the history in order to get the variance at each layer
    for (int layer = 0; layer < layers; layer++)
    {
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);
        unsigned short * matrixCell = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

        // This is less then or equal to due to a round error that can give +- 1 offsets
        if (abs((int) *matrixCell - resetOffset) <= 2)
        {
            dontUseAlphaYet = true;
        }

        sum += (int) *matrixCell;
    }

    unsigned short avg = sum / layers;

    // This is the latest height map
    unsigned short * heightValue = (unsigned short*)((char*) newestHeightMap + indexY * pitchNewestHeightMap) + indexX;
    int heightValueInt = (int) *heightValue;

    unsigned short *outputPointer = (unsigned short *)((char*) resultPointer + indexY * pitchResult) + indexX;

    // We have an initial guess at all the height values and if the previous height value is that guess, don't apply the alpha, just accept the new value
    if (dontUseAlphaYet)
    {
        *outputPointer = *heightValue;
        return;
    }

    float diff = (float) heightValueInt - avg;
    float newVariance = abs(diff);

    float heightEstimate = (float) *heightValue * alpha + (avg * (1.0 - alpha)); // (newHeight * newVariance) / newVariance;

    *outputPointer = (unsigned short) heightEstimate;
}
