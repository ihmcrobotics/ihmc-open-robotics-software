extern "C"
__global__ void element_wise_add(uint16 * matrixA, uint16 * matrixB, uint16 * result, int width, int height, )
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int strideX = blockDim.x * gridDim.x;

    int indexY = blockIdx.y * blockDim.y + threadIdx.y;
    int strideY = blockDim.y * gridDim.y;

    for (int x = indexX; x < width; x += strideX)
    {
        for (int y = indexY; y < height; y += strideY)
        {
            result[x][y] = matrixA[x][y] + matrixB[x][y];
        }
    }
}
