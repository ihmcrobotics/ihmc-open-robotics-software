// https://hertasecurity.com/wp-content/uploads/work-efficient-parallel-non-maximum-suppression.pdf

#include "Utils.cu"

typedef struct
{
    float x, y, width, height, score;
} box;

extern "C"
__global__ void checkInclusion(box* boxes, size_t boxCount, float overlapThreshold, bool* inclusionMatrix)
{
    int startX = Utils::getThreadCoordX();
    int startY = Utils::getThreadCoordY();

    int strideX = Utils::getStrideX();
    int strideY = Utils::getStrideY();

    // Declare variables
    box boxI;
    box boxJ;

    float boxIArea;
    float boxJArea;
    float intersectionWidth;
    float intersectionHeight;
    float intersectionArea;
    float intersectionOverUnion;

    for (int i = startY; i < boxCount; i += strideY)
    {
        for (int j = startX; j < boxCount; j += strideX)
        {
            bool keep = true;

            if (i == j)
            {
                inclusionMatrix[i * boxCount + j] = keep;
                continue;
            }

            boxI = boxes[i];
            boxJ = boxes[j];

            boxIArea = (boxI.width + 1.0f) * (boxI.height + 1.0f);
            boxJArea = (boxJ.width + 1.0f) * (boxJ.height + 1.0f);

            intersectionWidth = max(0.0f, min(boxI.x + boxI.width, boxJ.x + boxJ.width) - max(boxI.x, boxJ.x) + 1.0f);
            intersectionHeight = max(0.0f, min(boxI.y + boxI.height, boxJ.y + boxJ.height) - max(boxI.y, boxJ.y) + 1.0f);
            intersectionArea = intersectionWidth * intersectionHeight;

            intersectionOverUnion = intersectionArea / (boxIArea + boxJArea - intersectionArea);

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
__global__ void reduceFast(bool* inclusionMatrix, size_t boxCount, bool* inclusionVector)
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

    inclusionVector[row] = __syncthreads_and(inclusionMatrix[row * boxCount + column]);
}

/*
 * A slower, but more versatile reduction kernel.
 * Works on any number of boxes.
 */
extern "C"
__global__ void reduceSlow(bool* inclusionMatrix, size_t boxCount, bool* inclusionVector)
{
    int row = Utils::getThreadCoordX();

    // Bounds check
    if (row >= boxCount)
        return;

    bool keep = true;
    for (int column = 0; column < boxCount && keep; ++column)
        keep &= inclusionMatrix[row * boxCount + column];

    inclusionVector[row] = keep;
}