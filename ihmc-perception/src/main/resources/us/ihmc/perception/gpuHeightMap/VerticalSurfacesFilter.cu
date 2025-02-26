extern "C"
__global__ void removeCinderBlockWalls(unsigned short * matrixPointer, size_t pitchMatrix,
                                       unsigned short * resultPointer, size_t pitchResult,
                                       int width, int height, int threshold)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    // Access current height value using pitched memory
    unsigned short *row = (unsigned short *)((char *)matrixPointer + y * pitchMatrix);
    unsigned short currentHeight = row[x];

    // Array to store valid neighbor values
    unsigned short neighbors[8];
    int neighborCount = 0;

    // Define neighbor offsets (8-connected)
    int dx[8] = { 0,  0, -1,  1, -1,  1, -1,  1 };
    int dy[8] = {-1,  1,  0,  0, -1, -1,  1,  1 };

    // Find min and max among valid neighbors
    unsigned short minNeighbor = currentHeight;
    unsigned short maxNeighbor = currentHeight;

    for (int i = 0; i < 8; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        // Check bounds
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            unsigned short *neighborRow = (unsigned short *)((char *)matrixPointer + ny * pitchMatrix);
            unsigned short neighborHeight = neighborRow[nx];

            neighbors[neighborCount++] = neighborHeight;
            if (neighborHeight < minNeighbor) minNeighbor = neighborHeight;
            if (neighborHeight > maxNeighbor) maxNeighbor = neighborHeight;
        }
    }

    // Only adjust if the height jump is significant (above threshold)
    if ((currentHeight > minNeighbor + threshold) && (currentHeight < maxNeighbor - threshold)) {
        // Adjust to the closest level
        if (abs(currentHeight - minNeighbor) < abs(currentHeight - maxNeighbor)) {
            currentHeight = minNeighbor;
        } else {
            currentHeight = maxNeighbor;
        }
    }

    // Store result in pitched output matrix
    unsigned short *resultRow = (unsigned short *)((char *)resultPointer + y * pitchResult);
    resultRow[x] = currentHeight;
}