/*
 * Kernels for post processing YOLOv8 output
 *
 * Unfiltered data contains COLUMNS of floats as such:
 *  - 4 floats:  bounding box dimensions (center X, center Y, width, height)
 *  - n floats:  confidence values, one for each object class the model can detect
 *  - 32 floats: mask weights
 *
 * Filtered data contains ROWS of floats as such:
 *  - 4 floats:  bounding box dimensions (x, y, width, height)
 *  - 1 float:   confidence
 *  - 1 float:   object class index
 *  - 32 floats: mask weights
 */

#define X_INDEX 0
#define Y_INDEX 1
#define WIDTH_INDEX 2
#define HEIGHT_INDEX 3
#define CONFIDENCE_INDEX 4
#define CLASS_INDEX 5

#define FILTERED_FLOATS_PER_ROW 38
#define UNFILTERED_FLOATS_PER_COLUMN(n) (4 + n + 32)

#include "Utils.cu"

template<typename T>
__device__ T* row(const T* matrix, long pitch, int row)
{
    return (T*)((char*) matrix + pitch * row);
}

template<typename T>
__device__ T* col(const T* matrix, int column)
{
    return (T*)(matrix + column);
}

extern "C"
__global__ void filterDetections(float* unfilteredDetection, int classCount, int detectionCount, float confidenceThreshold, float* filteredDetections, int* filteredDetectionCount)
{
    if (Utils::getThreadCoordX() == 0)
        *filteredDetectionCount = 0;
    __syncthreads();

    int start = Utils::getThreadCoordX();
    int stride = Utils::getStrideX();

    for (int i = start; i < detectionCount; i += stride)
    {
        // Get a pointer to the unfiltered column we want to process
        float* unfilteredColumn = col(unfilteredDetection, i);

        // Find the class with maximum confidence, and the confidence value
        int mostConfidentClass = 0; // class 0
        float maxConfidence = *row(unfilteredColumn, sizeof(float) * detectionCount, CONFIDENCE_INDEX);
        for (int classIndex = 1; classIndex < classCount; ++classIndex)
        {
            float confidence = *row(unfilteredColumn, sizeof(float) * detectionCount, CONFIDENCE_INDEX + classIndex);
            if (confidence > maxConfidence)
            {
                mostConfidentClass = classIndex;
                maxConfidence = confidence;
            }
        }

        // If the confidence is below our threshold, we skip this detection
        if (maxConfidence < confidenceThreshold)
            continue;

        // Get the next row index, and atomically increment the filteredDetectionCount by 1
        int filteredRowIndex = atomicAdd(filteredDetectionCount, 1);

        // Get values
        float width = *row(unfilteredColumn, sizeof(float) * detectionCount, WIDTH_INDEX);
        float height = *row(unfilteredColumn, sizeof(float) * detectionCount, HEIGHT_INDEX);
        float x = *row(unfilteredColumn, sizeof(float) * detectionCount, X_INDEX) - width / 2.0f;
        float y = *row(unfilteredColumn, sizeof(float) * detectionCount, Y_INDEX) - height / 2.0f;

        // Assign the values to the filtered column
        float* filteredRow = row(filteredDetections, sizeof(float) * FILTERED_FLOATS_PER_ROW, filteredRowIndex);
        filteredRow[X_INDEX] = x;
        filteredRow[Y_INDEX] = y;
        filteredRow[WIDTH_INDEX] = width;
        filteredRow[HEIGHT_INDEX] = height;
        filteredRow[CONFIDENCE_INDEX] = maxConfidence;
        filteredRow[CLASS_INDEX] = (float) mostConfidentClass;

        // Copy mask weights over
        for (int j = 0; j < 32; ++j)
        {
            filteredRow[6 + j] = *row(unfilteredColumn, sizeof(float) * detectionCount, 4 + classCount + j);
        }
    }
}