// Define neighbor offsets (8-connected)
const int dx[8] = { 0,  0, -1,  1, -1,  1, -1,  1 };
const int dy[8] = {-1,  1,  0,  0, -1, -1,  1,  1 };

extern "C"
__global__ void removeCinderBlockWalls(unsigned short * matrixPointer, size_t pitchMatrix,
                                       unsigned short * resultPointer, size_t pitchResult,
                                       int width, int height, int threshold)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height)
        return;

    unsigned short *row = (unsigned short *)((char *)matrixPointer + y * pitchMatrix);
    unsigned short currentHeight = row[x];

    unsigned short neighbors[8];
    int neighborCount = 0;

    unsigned short minNeighbor = currentHeight;
    unsigned short maxNeighbor = currentHeight;

    for (int i = 0; i < 8; i++)
    {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
        {
            unsigned short *neighborRow = (unsigned short *)((char *)matrixPointer + ny * pitchMatrix);
            unsigned short neighborHeight = neighborRow[nx];

            neighbors[neighborCount++] = neighborHeight;
            if (neighborHeight < minNeighbor) minNeighbor = neighborHeight;
            if (neighborHeight > maxNeighbor) maxNeighbor = neighborHeight;
        }
    }

    // Only adjust if the height jump is significant (above threshold)
    if ((currentHeight > minNeighbor + threshold) && (currentHeight < maxNeighbor - threshold))
    {
        if (abs(currentHeight - minNeighbor) < abs(currentHeight - maxNeighbor))
        {
            currentHeight = minNeighbor;
        }
        else
        {
            currentHeight = maxNeighbor;
        }
    }

    unsigned short *resultRow = (unsigned short *)((char *)resultPointer + y * pitchResult);
    resultRow[x] = currentHeight;
}