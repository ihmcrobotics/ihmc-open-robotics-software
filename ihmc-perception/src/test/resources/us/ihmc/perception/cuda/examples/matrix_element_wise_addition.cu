extern "C"
__global__
void element_wise_add(unsigned short * matrixA, size_t pitchA,
                      unsigned short * matrixB, size_t pitchB,
                      unsigned short * result, size_t pitchResult,
                      int rows, int cols)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int strideX = blockDim.x * gridDim.x;

    int indexY = blockIdx.y * blockDim.y + threadIdx.y;
    int strideY = blockDim.y * gridDim.y;

    for (int y = indexY; y < rows; y += strideY)
    {
        unsigned short * matrixARow = (unsigned short*)((char*) matrixA + y * pitchA);
        unsigned short * matrixBRow = (unsigned short*)((char*) matrixB + y * pitchB);
        unsigned short * resultRow = (unsigned short*)((char*) result + y * pitchResult);

        for (int x = indexX; x < cols; x += strideX)
        {
            resultRow[x] = matrixARow[x] + matrixBRow[x];
        }
    }
}
