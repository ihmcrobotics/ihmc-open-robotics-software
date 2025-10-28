#include "Utils.cu"

typedef struct
{
    float x, y, width, height, score;
} Box;

extern "C"
__global__ void checkInclusion(Box* boxes, int boxCount, float overlapThreshold, bool* inclusionMatrix)
{
    int startX = Utils::getThreadCoordX();
    int startY = Utils::getThreadCoordY();

    int strideX = Utils::getStrideX();
    int strideY = Utils::getStrideY();

    // Declare variables
    Box boxI;
    Box boxJ;

    float boxIArea;
    float boxJArea;
    float intersectionWidth;
    float intersectionHeight;
    float intersectionArea;
    float intersectionOverUnion;

    for (int i = startY; i < boxCount; i += strideY)
    {
        // Box is removed if its area is 0 (it's a line, or a point... not a box)
        if (boxes[i].width == 0.0f || boxes[i].height == 0.0f)
        {
            inclusionMatrix[i * boxCount] = false;
            continue;
        }

        // Compare box against all other boxes
        for (int j = startX; j < boxCount; j += strideX)
        {
            if (i == j) // No need to compare against itself
            {
                inclusionMatrix[i * boxCount + j] = true;
                continue;
            }

            bool keep = true;

            // Find IoU
            boxI = boxes[i];
            boxJ = boxes[j];

            boxIArea = (boxI.width + 1.0f) * (boxI.height + 1.0f);
            boxJArea = (boxJ.width + 1.0f) * (boxJ.height + 1.0f);

            intersectionWidth = max(0.0f, min(boxI.x + boxI.width, boxJ.x + boxJ.width) - max(boxI.x, boxJ.x) + 1.0f);
            intersectionHeight = max(0.0f, min(boxI.y + boxI.height, boxJ.y + boxJ.height) - max(boxI.y, boxJ.y) + 1.0f);
            intersectionArea = intersectionWidth * intersectionHeight;

            intersectionOverUnion = intersectionArea / (boxIArea + boxJArea - intersectionArea);

            // If the box overlaps another and its score is lower than the other, remove it
            if (intersectionOverUnion > overlapThreshold && boxI.score < boxJ.score)
                keep = false;

            inclusionMatrix[i * boxCount + j] = keep;
        }
    }
}

/*
 * A fast kernel for reducing the inclusion matrix into a vector.
 * Only works if the number of boxes is less than the maximum threads per block.
 * For larger reductions, use the slow kernel.
 *
 * Launch as 1D kernel, where
 *  - gridDim.x >= boxCount
 *  - blockDim.x >= boxCount
 */
extern "C"
__global__ void reduceFast(bool* inclusionMatrix, int boxCount, int* includedIndices, int* includedBoxCount)
{
    // Each box must have a dedicated thread to itself
    assert(boxCount <= blockDim.x);
    // Each box must have a dedicated block to itself
    assert(boxCount <= gridDim.x);

    // Each block operates on a row of the inclusion matrix
    int row = blockIdx.x;
    // Each thread operates on the cell (column, row) of the inclusion matrix
    int column = threadIdx.x;

    // Bounds check
    if (row >= boxCount || column >= boxCount)
        return;

    // If non of the columns indicate to exclude this detection matrix
    if (__syncthreads_and(inclusionMatrix[row * boxCount + column]))
    {
        // Use one thread to add this detection to included indices
        if (threadIdx.x == 0)
        {
            int nextIndex = atomicAdd(includedBoxCount, 1);
            includedIndices[nextIndex] = row;
        }
    }
}

/*
 * A slower, but more versatile reduction kernel.
 * Works on any number of boxes.
 */
extern "C"
__global__ void reduceSlow(bool* inclusionMatrix, int boxCount, int* includedIndices, int* includedBoxCount)
{
    int row = Utils::getThreadCoordX();

    // Bounds check
    if (row >= boxCount)
        return;

    bool include = true;
    for (int column = 0; column < boxCount && include; ++column)
        include &= inclusionMatrix[row * boxCount + column];

    if (include)
    {
        int nextIndex = atomicAdd(includedBoxCount, 1);
        includedIndices[nextIndex] = row;
    }
}