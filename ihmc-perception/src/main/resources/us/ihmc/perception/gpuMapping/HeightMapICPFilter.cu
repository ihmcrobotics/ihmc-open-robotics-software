#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define CELL_SIZE 0
#define CENTER_INDEX 1
#define CELLS_PER_AXIS 2
#define MIN_HEIGHT_REGISTRATION 9
#define MAX_HEIGHT_REGISTRATION 10
#define MIN_CLAMP_HEIGHT 11
#define MAX_CLAMP_HEIGHT 12
#define KALMAN_FILTER_PREDICTION_NOISE 13
#define ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED 14
#define VARIANCE_PER_METER 15
#define VARIANCE_PER_TRANSLATION_SPEED 16
#define VARIANCE_PER_ROTATION_SPEED 17
#define MIN_DEPTH_TO_ACCEPT 19

extern "C"
__global__ void heightMapICPKernel(float *__restrict__ localMap, size_t pitchLocal,
                                   float *__restrict__ globalMeanMap, size_t pitchGlobalMean,
                                   const float *__restrict__ localToGlobalTransform,
                                   const float globalMapCenterX,
                                   const float globalMapCenterY,
                                   float *__restrict__ vectorMap, size_t pitchvector,
                                   const float *__restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    if (xIndex >= params[CELLS_PER_AXIS] || yIndex >= params[CELLS_PER_AXIS])
        return;

    float2 localCoordinates = indices_to_coordinate(make_int2(xIndex, yIndex), make_float2(0.0f, 0.0f), params[CELL_SIZE], params[CENTER_INDEX]);

    float* localRow = (float*)((char*)localMap + xIndex * pitchLocal);
    float localHeight = localRow[yIndex];
    float3 localCoordinates3f = make_float3(localCoordinates.x, localCoordinates.y, localHeight);
    float3 localCoordinatesInGlobalFrame = transformPoint3D(make_float3(localCoordinates.x, localCoordinates.y, localHeight), localToGlobalTransform);

    int2 globalIndices = coordinate_to_indices(make_float2(localCoordinatesInGlobalFrame.x, localCoordinatesInGlobalFrame.y),
                                               make_float2(globalMapCenterX, globalMapCenterY),
                                               params[CELL_SIZE],
                                               params[CENTER_INDEX]);

    float3 minDistanceVector = make_float3(0.0f/0.0f, 0.0f/0.0f, 0.0f/0.0f);

    if (globalIndices.x >= 0 && globalIndices.x < params[CELLS_PER_AXIS] && globalIndices.y >= 0 && globalIndices.y < params[CELLS_PER_AXIS])
    {
        float* globalRow = (float*)((char*)globalMeanMap + globalIndices.x * pitchGlobalMean);
        float globalHeight = globalRow[globalIndices.y];
        float2 globalMapCellCoordinates = indices_to_coordinate(make_int2(globalIndices.x, globalIndices.y),
                                                                make_float2(globalMapCenterX, globalMapCenterY),
                                                                params[CELL_SIZE],
                                                                params[CENTER_INDEX]);
        float3 globalPoint = make_float3(globalMapCellCoordinates.x ,globalMapCellCoordinates.y, globalHeight);
        minDistanceVector = sub(globalPoint, localCoordinates3f);

        minDistanceVector.x = fabsf(minDistanceVector.x);
        minDistanceVector.y = fabsf(minDistanceVector.y);
        minDistanceVector.z = fabsf(minDistanceVector.z);
    }

    float* vectorRow = (float *)((char *)vectorMap + xIndex * pitchvector);

    vectorRow[3 * yIndex + 0] = minDistanceVector.x;
    vectorRow[3 * yIndex + 1] = minDistanceVector.y;
    vectorRow[3 * yIndex + 2] = minDistanceVector.z;
}