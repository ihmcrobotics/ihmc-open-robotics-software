#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define GLOBAL_CENTER_INDEX 2
#define LOCAL_CELLS_PER_AXIS 3
#define GLOBAL_CELLS_PER_AXIS 4
#define SEARCH_RADIUS 5

extern "C"
__global__ void heightMapICPKernelV2(float *__restrict__ localMap, size_t pitchLocal,
                                     float *__restrict__ globalMeanMap, size_t pitchGlobalMean,
                                     const float *__restrict__ localToGlobalTransform,
                                     const float globalMapCenterX,
                                     const float globalMapCenterY,
                                     float *__restrict__ vectorMap, size_t pitchvector,
                                     const float *__restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    if (xIndex >= params[LOCAL_CELLS_PER_AXIS] || yIndex >= params[LOCAL_CELLS_PER_AXIS])
        return;

    // Local cell → local coordinates
    float2 localCoordinates = indices_to_coordinate(
        make_int2(xIndex, yIndex),
        make_float2(0.0f, 0.0f),
        params[CELL_SIZE],
        params[LOCAL_CENTER_INDEX]);

    float* localRow = (float*)((char*)localMap + xIndex * pitchLocal);
    float localHeight = localRow[yIndex];

    // Transform local point into global frame
    float3 localCoordinatesInGlobalFrame =
        transformPoint3D(make_float3(localCoordinates.x, localCoordinates.y, localHeight),
                         localToGlobalTransform);

    // Global map indices (DIRECT correspondence — no search)
    int2 globalIndices = coordinate_to_indices(
        make_float2(localCoordinatesInGlobalFrame.x, localCoordinatesInGlobalFrame.y),
        make_float2(globalMapCenterX, globalMapCenterY),
        params[CELL_SIZE],
        params[GLOBAL_CENTER_INDEX]);

    float3 minDistanceVector = make_float3(
        0.0f/0.0f, 0.0f/0.0f, 0.0f/0.0f);

    // Must be interior for 3x3 neighbor computation
    if (globalIndices.x > 0 &&
        globalIndices.x < params[GLOBAL_CELLS_PER_AXIS] - 1 &&
        globalIndices.y > 0 &&
        globalIndices.y < params[GLOBAL_CELLS_PER_AXIS] - 1)
    {
        float* globalRowM1 = (float*)((char*)globalMeanMap + (globalIndices.x - 1) * pitchGlobalMean);
        float* globalRow0  = (float*)((char*)globalMeanMap + globalIndices.x * pitchGlobalMean);
        float* globalRowP1 = (float*)((char*)globalMeanMap + (globalIndices.x + 1) * pitchGlobalMean);

        // Compute finite differences along X and Y using 3x3 neighbors
        float dzdx = ((globalRowP1[globalIndices.y - 1] + 2.0f * globalRowP1[globalIndices.y] + globalRowP1[globalIndices.y + 1])
                     - (globalRowM1[globalIndices.y - 1] + 2.0f * globalRowM1[globalIndices.y] + globalRowM1[globalIndices.y + 1]))
                     / (8.0f * params[CELL_SIZE]);

        float dzdy = ((globalRowM1[globalIndices.y + 1] + 2.0f * globalRow0[globalIndices.y + 1] + globalRowP1[globalIndices.y + 1])
                     - (globalRowM1[globalIndices.y - 1] + 2.0f * globalRow0[globalIndices.y - 1] + globalRowP1[globalIndices.y - 1]))
                     / (8.0f * params[CELL_SIZE]);

        // Surface normal
        float3 normal = normalize(make_float3(-dzdx, -dzdy, 1.0f));

        // Height residual
        float zCenter = globalRow0[globalIndices.y];
        float heightResidual = zCenter - localCoordinatesInGlobalFrame.z;

        // Point-to-plane vector
        float signedDistance = heightResidual * normal.z;
        minDistanceVector = make_float3(
            normal.x * signedDistance,
            normal.y * signedDistance,
            normal.z * signedDistance);
    }

    // Store result
    float* vectorRow = (float *)((char *)vectorMap + xIndex * pitchvector);
    vectorRow[3 * yIndex + 0] = minDistanceVector.x;
    vectorRow[3 * yIndex + 1] = minDistanceVector.y;
    vectorRow[3 * yIndex + 2] = minDistanceVector.z;
}

extern "C"
__global__ void heightMapICPKernel(
    float *__restrict__ localMap, size_t pitchLocal,
    float *__restrict__ globalMeanMap, size_t pitchGlobalMean,
    const float *__restrict__ localToGlobalTransform,
    const float globalMapCenterX,
    const float globalMapCenterY,
    float *__restrict__ vectorMap, size_t pitchVector,
    unsigned char *__restrict__ validityMap, size_t pitchValidity,
    const float *__restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    if (xIndex >= params[LOCAL_CELLS_PER_AXIS] || yIndex >= params[LOCAL_CELLS_PER_AXIS])
        return;

    float2 localCoordinates = indices_to_coordinate(
        make_int2(xIndex, yIndex),
        make_float2(0.0f, 0.0f),
        params[CELL_SIZE],
        params[LOCAL_CENTER_INDEX]);

    // Local map
    float* localRow = (float*)((char*)localMap + yIndex * pitchLocal);
    float localHeight = localRow[xIndex];

    // Vector map
    float* vectorRow = (float*)((char*)vectorMap + yIndex * pitchVector);

    // Validity map
    unsigned char* validityRow = (unsigned char*)((char*)validityMap + yIndex * pitchValidity);

    if (localHeight == 0.0f)
    {
        // Mark as invalid
        validityRow[yIndex] = 0;
        vectorRow[3 * yIndex + 0] = 0.0f;
        vectorRow[3 * yIndex + 1] = 0.0f;
        vectorRow[3 * yIndex + 2] = 0.0f;
        return;
    }

    float3 localCoordinatesInGlobalFrame = transformPoint3D(
        make_float3(localCoordinates.x, localCoordinates.y, localHeight),
        localToGlobalTransform);

    int2 globalIndices = coordinate_to_indices(
        make_float2(localCoordinatesInGlobalFrame.x, localCoordinatesInGlobalFrame.y),
        make_float2(localToGlobalTransform[3], localToGlobalTransform[7]),
        params[CELL_SIZE],
        params[GLOBAL_CENTER_INDEX]);

    float minDistance = 1e9f;
    float3 closestGlobalPoint = make_float3(0.0f, 0.0f, 0.0f);
    bool foundValidCorrespondence = false;
    int searchRadius = params[SEARCH_RADIUS];

    for (int dx = -searchRadius; dx <= searchRadius; dx++)
    {
        for (int dy = -searchRadius; dy <= searchRadius; dy++)
        {
            int searchX = globalIndices.x + dx;
            int searchY = globalIndices.y + dy;

            if (searchX >= 0 && searchX < params[GLOBAL_CELLS_PER_AXIS] &&
                searchY >= 0 && searchY < params[GLOBAL_CELLS_PER_AXIS])
            {
                float* globalRow = (float*)((char*)globalMeanMap + searchY * pitchGlobalMean);
                float globalHeight = globalRow[searchX];
                float2 globalCellCoords = indices_to_coordinate(
                    make_int2(searchX, searchY),
                    make_float2(localToGlobalTransform[3], localToGlobalTransform[7]),
                    params[CELL_SIZE],
                    params[GLOBAL_CENTER_INDEX]);

                float3 candidatePoint = make_float3(globalCellCoords.x, globalCellCoords.y, globalHeight);
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

    if (foundValidCorrespondence)
    {
        float3 minDistanceVector = sub(closestGlobalPoint, localCoordinatesInGlobalFrame);
        vectorRow[3 * yIndex + 0] = minDistanceVector.x;
        vectorRow[3 * yIndex + 1] = minDistanceVector.y;
        vectorRow[3 * yIndex + 2] = minDistanceVector.z;
        validityRow[yIndex] = 1; // valid
    }
    else
    {
        vectorRow[3 * yIndex + 0] = 0.0f;
        vectorRow[3 * yIndex + 1] = 0.0f;
        vectorRow[3 * yIndex + 2] = 0.0f;
        validityRow[yIndex] = 0; // invalid
    }
}
