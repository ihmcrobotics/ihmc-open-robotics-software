/*
 * Kernels for post processing YOLOv8 output
 *
 * Unfiltered data contains columns of floats as such:
 *  - 4 floats:  bounding box dimensions (center X, center Y, width, height)
 *  - n floats:  confidence values, one for each object class the model can detect
 *  - 32 floats: mask weights
 *
 * Filtered data contains columns of floats as such:
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

#define FILTERED_FLOATS_PER_COLUMN 38
#define UNFILTERED_FLOATS_PER_COLUMN(n) (4 + n + 32)

#include "Utils.cu"

extern "C"
__global__ void filterDetections(float* unfilteredDetection, int classCount, size_t detectionCount, float confidenceThreshold, float* filteredDetections, size_t* filteredDetectionCount)
{
    if (Utils::getThreadCoordX == 0)
    {
        *filteredDetectionCount = 0;
    }
    __syncthreads();

    int start = Utils::getThreadCoordX();
    int stride = Utils::getStrideX();

    for (int i = start; i < detectionCount; i += stride)
    {
        // Get a pointer to the unfiltered column we want to process
        float* unfilteredColumn = unfilteredDetection + (i * UNFILTERED_FLOATS_PER_COLUMN(classCount));

        // Find the class with maximum confidence, and the confidence value
        float mostConfidentClass = 0; // class 0
        float maxConfidence = unfilteredColumn[4];
        for (int classIndex = 1; classIndex < classCount; ++classIndex)
        {
            float confidence = unfilteredColumn[4 + i];
            if (confidence > maxConfidence)
            {
                mostConfidentClass = i;
                maxConfidence = confidence;
            }
        }

        // If the confidence is below our threshold, we skip this detection
        if (maxConfidence < confidenceThreshold)
            continue;

        // Get the next column index, and atomically increment the filteredDetectionCount by 1
        int filteredColumnIndex = atomicAdd(filteredDetectionCount, 1);

        // Get values
        int width = unfilteredColumn[WIDTH_INDEX];
        int height = unfilteredColumn[HEIGHT_INDEX];
        int x = unfilteredColumn[X_INDEX] - width / 2.0f;
        int y = unfilteredColumn[Y_INDEX] - height / 2.0f

        // Assign the values to the filtered column
        float* filteredColumn = filteredDetections + (filteredColumnIndex * FILTERED_FLOATS_PER_COLUMN);
        filteredColumn[X_INDEX] = x;
        filteredColumn[Y_INDEX] = y;
        filteredColumn[WIDTH_INDEX] = width;
        filteredColumn[HEIGHT_INDEX] = height;
        filteredColumn[CLASS_INDEX] = mostConfidentClass;
        filteredColumn[CONFIDENCE_INDEX] = maxConfidence;

        // Copy mask weights over
        for (int j = 0; j < 32; ++j)
        {
            filteredColumn[6 + j] = unfilteredColumn[4 + classCount + j];
        }
    }
}