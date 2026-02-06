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
#include "PerceptionUtils.cu"

using namespace PerceptionUtils;

extern "C"
__global__ void filterDetections(float* unfilteredDetection,
                                 int classCount,
                                 int detectionCount,
                                 float* confidenceThresholds,
                                 bool* ignoredClasses,
                                 float* filteredDetections,
                                 int* filteredDetectionCount)
{
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

        // If the class is ignored or confidence is below the threshold, we skip this detection
        if (ignoredClasses[mostConfidentClass] || maxConfidence < confidenceThresholds[mostConfidentClass])
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

extern "C"
__global__ void computeDetectionMask(float* prototypeMasks,
                                     float* weights,
                                     float* boundingBox,
                                     float maskThreshold,
                                     unsigned char* mask,
                                     size_t maskPitch,
                                     int maskWidth,
                                     int maskHeight)
{
    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    const float boundingBoxLeft = 0.25f * boundingBox[0];                       // X
    const float boundingBoxRight = 0.25f * (boundingBox[0] + boundingBox[2]);   // X + width
    const float boundingBoxTop = 0.25f * boundingBox[1];                        // Y
    const float boundingBoxBottom = 0.25f * (boundingBox[1] + boundingBox[3]);  // Y + height

    float* prototypeMask;
    float prototypeValue;
    float floatMaskValue;
    unsigned char maskValue;

    for (int y = startY; y < maskHeight; y += strideY)
    {
        for (int x = startX; x < maskWidth; x += strideX)
        {
            maskValue = 0x00;

            // If we're in the bounding box
            if (!(x < boundingBoxLeft || x > boundingBoxRight ||
                  y < boundingBoxTop || y > boundingBoxBottom))
            {
                // Calculate float mask value
                floatMaskValue = 0.0f;
                for (int i = 0; i < 32; ++i)
                {
                    prototypeMask = prototypeMasks + i * maskWidth * maskHeight;
                    prototypeValue = prototypeMask[y * maskWidth + x];
                    floatMaskValue += weights[i] * prototypeValue;
                }

                // If the value is greater than the threshold, we have a valid point!
                if (floatMaskValue > maskThreshold)
                    maskValue = 0xFF;
            }

            *col(row(mask, maskPitch, y), x) = maskValue;
        }
    }
}