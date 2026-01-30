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

    float2 localCoordinates = indices_to_coordinate(make_int2(xIndex, yIndex), make_float2(0.0f, 0.0f), params[CELL_SIZE], params[CENTER_INDEX]);

    float* localRow = (float*)((char*)localMap + xIndex * pitchLocal);
    float localHeight = localRow[yIndex];
    float3 localCoordinatesInGlobalFrame = transformPoint3D(make_float3(localCoordinates.x, localCoordinates.y, localHeight), localToGlobalTransform);

    int2 globalIndices = coordinate_to_indices(make_float2(localCoordinatesInGlobalFrame.x, localCoordinatesInGlobalFrame.y),
                                               make_float2(globalMapCenterX, globalMapCenterY),
                                               params[CELL_SIZE],
                                               params[CENTER_INDEX]);

//     float minDistance = 100000.0f;
    float3 minDistanceVector = make_float3(0.0f, 0.0f, 0.0f);

    if (globalIndices.x >= 0 && globalIndices.x < params[CELLS_PER_AXIS] && globalIndices.y >= 0 && globalIndices.y < params[CELLS_PER_AXIS])
    {
        float* globalRow = (float*)((char*)globalMeanMap + globalIndices.x * pitchGlobalMean);
        float globalHeight = globalRow[globalIndices.y];
        float2 globalMapCellCoordinates = indices_to_coordinate(make_int2(globalIndices.x, globalIndices.y),
                                                                make_float2(globalMapCenterX, globalMapCenterY),
                                                                params[CELL_SIZE],
                                                                params[CENTER_INDEX]);
        float3 globalPoint = make_float3(globalMapCellCoordinates.x ,globalMapCellCoordinates.y, globalHeight);
        minDistanceVector = sub(globalPoint, localCoordinatesInGlobalFrame);
//         float distancef = distance(localCoordinatesInGlobalFrame, temp);


    }
//     for (int i = 0; i < params[CELLS_PER_AXIS]; i++)
//     {
//         for (int j = 0; j < params[CELLS_PER_AXIS]; j++)
//         {
//             float2 globalMapCellCoordinates = indices_to_coordinate(make_int2(i, j), make_float2(globalMapCenterX, globalMapCenterY), params[CELL_SIZE], params[CENTER_INDEX]);
//             float* globalRow = (float*)((char*)globalMeanMap + i * pitchGlobalMean);
//             float globalHeight = globalRow[j];
//             float3 temp = make_float3(globalMapCellCoordinates.x ,globalMapCellCoordinates.y, globalHeight);
//             float distancef = distance(localCoordinatesInGlobalFrame, temp);
//
//             if (distancef < minDistance)
//             {
//                 minDistance = distancef;
//                 minDistanceVector = sub(temp, localCoordinatesInGlobalFrame);
//             }
//         }
//     }

    float* vectorRow = (float *)((char *)vectorMap + xIndex * pitchvector);

    vectorRow[3 * yIndex + 0] = minDistanceVector.x;
    vectorRow[3 * yIndex + 1] = minDistanceVector.y;
    vectorRow[3 * yIndex + 2] = minDistanceVector.z;
}