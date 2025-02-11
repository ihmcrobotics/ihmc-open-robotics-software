extern "C"
__global__
void filterRapidHeightMap(unsigned short * matrixPointer, size_t pitchA,
                         unsigned short * resultPointer, size_t pitchResult,
                         size_t layerSize, int rows, int cols, int layers)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    if (indexX >= cols || indexY >= rows)
        return;

    int sum = 0;

    for (int layer = 0; layer < layers; layer++)
    {
        // Compute the base address of the current layer
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);

        // Compute row offset using pitchA
        unsigned short * matrixPointerRow = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

        printf("Layer: %d, Value: %d, %d, and %d\n", layer, indexY, indexX, (int) *matrixPointerRow);
        sum += (int) *matrixPointerRow;
    }

    unsigned short avg = sum / layers;

    unsigned short *outputPointer = (unsigned short *)((char*) resultPointer + indexY * pitchResult) + indexX;
    *outputPointer = avg;
    printf("GPU Average: %d, ", avg);
}

