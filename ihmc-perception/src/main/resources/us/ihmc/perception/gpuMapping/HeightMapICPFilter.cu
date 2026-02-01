#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define CELL_SIZE 0
#define CENTER_INDEX 1
#define CELLS_PER_AXIS 2
#define SEARCH_RADIUS 3

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
    float3 localCoordinatesInGlobalFrame = transformPoint3D(make_float3(localCoordinates.x, localCoordinates.y, localHeight), localToGlobalTransform);

    int2 globalIndices = coordinate_to_indices(make_float2(localCoordinatesInGlobalFrame.x, localCoordinatesInGlobalFrame.y),
                                               make_float2(globalMapCenterX, globalMapCenterY),
                                               params[CELL_SIZE],
                                               params[CENTER_INDEX]);

    float minDistance = 1000000000; // Set to very large value to start
    float3 closestGlobalPoint = make_float3(0.0f/0.0f, 0.0f/0.0f, 0.0f/0.0f);

    bool foundValidCorrespondence = false;

    // Get search radius (default to 2 if not provided)
    int searchRadius = params[SEARCH_RADIUS];

    // Search in neighborhood around the transformed point
    for (int dx = -searchRadius; dx <= searchRadius; dx++)
    {
        for (int dy = -searchRadius; dy <= searchRadius; dy++)
        {
            int searchX = globalIndices.x + dx;
            int searchY = globalIndices.y + dy;

            // Check if search indices are within bounds
            if (searchX >= 0 && searchX < params[CELLS_PER_AXIS] && searchY >= 0 && searchY < params[CELLS_PER_AXIS])
            {
                float* globalRow = (float*)((char*)globalMeanMap + searchX * pitchGlobalMean);
                float globalHeight = globalRow[searchY];
                float2 globalMapCellCoordinates = indices_to_coordinate(make_int2(searchX, searchY),
                                                                        make_float2(globalMapCenterX, globalMapCenterY),
                                                                        params[CELL_SIZE],
                                                                        params[CENTER_INDEX]);
                float3 candidatePoint = make_float3(globalMapCellCoordinates.x ,globalMapCellCoordinates.y, globalHeight);

                float distF = distance(candidatePoint, localCoordinatesInGlobalFrame);

                if (distF < minDistance)
                {
                    minDistance = distF;
                    closestGlobalPoint = candidatePoint;
                    foundValidCorrespondence = true;

                }
            }
        }
    }

    // Compute the correspondence vector
    float3 minDistanceVector = make_float3(0.0f/0.0f, 0.0f/0.0f, 0.0f/0.0f);

    if (foundValidCorrespondence)
    {
        // Vector from local point (in global frame) to closest global point
        minDistanceVector = sub(closestGlobalPoint, localCoordinatesInGlobalFrame);
    }
    // else: leave as NaN to indicate no valid correspondence

    float* vectorRow = (float *)((char *)vectorMap + xIndex * pitchvector);

    vectorRow[3 * yIndex + 0] = minDistanceVector.x;
    vectorRow[3 * yIndex + 1] = minDistanceVector.y;
    vectorRow[3 * yIndex + 2] = minDistanceVector.z;
}