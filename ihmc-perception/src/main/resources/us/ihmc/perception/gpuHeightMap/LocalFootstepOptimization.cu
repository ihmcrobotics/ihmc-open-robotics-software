#include <cuda_runtime.h>
#include <math.h>

#define GRID_RESOLUTION_XY 0.02f
#define GRID_RESOLUTION_YAW 0.0872664626f // 5 degrees in radians
#define POSITION_W 10.0f
#define YAW_W 50.0f
#define PLANARITY_W 70.0f
#define HEIGHT_W 20.0f
#define MAX_HEIGHT_VARIANCE 0.05f
#define CONTINUOUS_SURFACE_TOLERANCE 0.02f
#define MAX_SLOPE_ANGLE 0.872664626f // approx 50 degrees in radians
#define DISCONTINUITY_PENALTY 10.0f
#define HEIGHT_CONSTRAINT_PENALTY 5.0f
#define SLOPE_CONSTRAINT_PENALTY 10.0f

extern "C" __global__ void optimizeFootstep(
    float* heightMapData,
    int width,
    int height,
    float resolution,
    float* initialPose,
    float searchRadius,
    int stepsXY,
    int stepsYaw,
    float* costs,
    float* solutions)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int totalThreads = stepsXY * stepsXY * stepsYaw;

    if (idx < totalThreads)
    {
        // Create a mapping from a 1D space (thread index) to a 3D space (x, y, yaw),

        // Divides the thread index by the total number of steps in the YZ plane
        int i = idx / (stepsXY * stepsYaw);
        // Divide idx by stepsYaw to get the position within the XY plane,
        // then uses the modulo operator to find its position within the Y direction of that plane.
        int j = (idx / stepsYaw) % stepsXY;
        // Use the modulo operator to find the remainder when divided by stepsYaw
        int k = idx % stepsYaw;

        float x = initialPose[0] - searchRadius + i * GRID_RESOLUTION_XY;
        float y = initialPose[1] - searchRadius + j * GRID_RESOLUTION_XY;
        float yaw = initialPose[2] + k * GRID_RESOLUTION_YAW;

        float cost = computeCost(x, y, yaw, initialPose, heightMapData, width, height, resolution);

        // Write the computed cost to the global memory array
        costs[idx] = cost;
        // Write the solution (x, y, yaw) to the global memory array
        solutions[idx * 3] = x;
        solutions[idx * 3 + 1] = y;
        solutions[idx * 3 + 2] = yaw;
    }
}

__global__ void findBestSolution(float* costs, float* solutions, int totalThreads, float* bestCost, float* bestSolution)
{
    __shared__ float sharedCosts[256];
    __shared__ int sharedIndices[256];

    int tid = threadIdx.x;
    int gid = blockIdx.x * blockDim.x + threadIdx.x;

    // Load data into shared memory
    if (gid < totalThreads)
    {
        sharedCosts[tid] = costs[gid];
        sharedIndices[tid] = gid;
    }
    else
    {
        sharedCosts[tid] = FLT_MAX;
        sharedIndices[tid] = -1;
    }
    __syncthreads();

    // Perform reduction in shared memory
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1)
    {
        if (tid < stride && sharedCosts[tid + stride] < sharedCosts[tid])
        {
            sharedCosts[tid] = sharedCosts[tid + stride];
            sharedIndices[tid] = sharedIndices[tid + stride];
        }
        __syncthreads();
    }

    // Write result for this block to global memory
    if (tid == 0)
    {
        atomicMin((int*)bestCost, __float_as_int(sharedCosts[0]));
        if (*bestCost == sharedCosts[0])
        {
            int index = sharedIndices[0];
            bestSolution[0] = solutions[index * 3];
            bestSolution[1] = solutions[index * 3 + 1];
            bestSolution[2] = solutions[index * 3 + 2];
        }
    }
}

// -------------------------------------------------------------------------
// Returns the height at world coordinates (x,y) given a height map that is
// stored as a contiguous float array. (The host must prepare the heightMap array
// and also provide the map dimensions, resolution, and origin.)
__device__ float getHeightAt(float x, float y,
const float* heightMap,
int mapWidth, int mapHeight,
float mapResolution, float mapOriginX, float mapOriginY)
{
int col = (int)((x - mapOriginX) / mapResolution);
int row = (int)((y - mapOriginY) / mapResolution);
// Clamp indices to valid bounds.
if(col < 0) col = 0;
if(col >= mapWidth) col = mapWidth - 1;
if(row < 0) row = 0;
if(row >= mapHeight) row = mapHeight - 1;
return heightMap[row * mapWidth + col];
}

__device__ float computeCost(float x, float y, float yaw, float* initialPose, float* heightMapData, int width, int height, float resolution)
{
    float positionCost = POSITION_W * (fabsf(x - initialPose[0]) + fabsf(y - initialPose[1]));
    float yawCost = YAW_W * fabsf(yaw - initialPose[2]);
    float planarityCost = PLANARITY_W * computePlanarityCost(x, y, yaw, heightMapData, width, height, resolution);

    float z = getHeightAt(x, y, heightMapData, width, height, resolution);
    float zDistancePenalty = HEIGHT_W * fabsf(z - initialPose[3]);

    return positionCost + yawCost + planarityCost + zDistancePenalty;
}

// -------------------------------------------------------------------------
// Computes the planarity cost for a candidate footstep pose. In this function
// we sample 5 points (the four corners of the foot and its center), then if the
// surface appears discontinuous we return a heavy penalty; otherwise, we discard
// the worst (most deviant) sample, fit a plane to remaining points and compute rmse
// height deviation and inclination
__device__ float computePlanarityCost(float x, float y, float yaw, float* heightMapData, int width, int height, float resolution)
{
    float fx = FOOT_LENGTH + 0.02f;
    float fy = FOOT_WIDTH + 0.02f;
    float fullSamples[13][2] = {{-fx/2, -fy/2}, {fx/2, -fy/2}, {fx/2, fy/2}, {-fx/2, fy/2}, {0, 0},
                                    {-fx/4, -fy/2}, {-fx/4, 0}, {-fx/4, fy/2}, {0, -fy/2}, {0, fy/2},
                                    {fx/4, -fy/2}, {fx/4, 0}, {fx/4, fy/2}};
    float fullSamplesXY[13][2];
    float heights[13];
    float cornersXY[5][2];

    // Sample points
    for (int i = 0; i < sizeof(otherSamplesXY); i++)
    {
        float sampleX = x + fullSamples[i][0] * cosf(yaw) - fullSamples[i][1] * sinf(yaw);
        float sampleY = y + fullSamples[i][0] * sinf(yaw) + fullSamples[i][1] * cosf(yaw);
        fullSamplesXY[i][0] = sampleX;
        fullSamplesXY[i][1] = sampleY;
        if (i < 5)
        {
            cornersXY[i][0] = sampleX;
            cornersXY[i][1] = sampleY;
        }
        fullHeights[i] = getHeightAt(sampleX, sampleY, heightMapData, width, height, resolution);
    }


    // Check for surface discontinuity
    float backHeightAvg = (heights[0] + heights[3]) / 2.0f;
    float frontHeightAvg = (heights[1] + heights[2]) / 2.0f;
    if (fabsf(heights[4] - (frontHeightAvg + backHeightAvg) / 2.0f) > CONTINUOUS_SURFACE_TOLERANCE)
    {
        float diff1 = fabsf(heights[0] - heights[4]);
        float diff2 = fabsf(heights[1] - heights[4]);
        float diff3 = fabsf(heights[2] - heights[4]);
        float diff4 = fabsf(heights[3] - heights[4]);
        return DISCONTINUITY_PENALTY * (diff1 + diff2 + diff3 + diff4);
    }

    // Fit plane and calculate planarity cost
    float planeCoeffs[4];
    fitPlane(fullSamplesXY, heights, planeCoeffs);

    float sumSquaredDistances = 0.0f;
    float maxVariance = 0.0f;
    float normalizer = sqrtf(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + 1); // Since C = -1

    for (int i = 0; i < size_of(heights); i++)
    {
        float distance = fabsf(planeCoeffs[0] * fullSamplesXY[i][0] + planeCoeffs[1] * fullSamplesXY[i][1]
                               - heights[i] + planeCoeffs[3]) / normalizer;
        sumSquaredDistances += distance * distance;
        maxVariance = fmaxf(maxVariance, distance);
    }

    float penalties = 0.0f;
    if (maxVariance > MAX_HEIGHT_VARIANCE)
    {
        penalties += HEIGHT_CONSTRAINT_PENALTY;
    }

    float rmse = sqrtf(sumSquaredDistances / 5.0f);
    float maxSlope = sqrtf(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1]);
    if (maxSlope > MAX_SLOPE_ANGLE)
    {
        penalties += SLOPE_CONSTRAINT_PENALTY;
    }

    return rmse + maxSlope + penalties;
}

__device__ void fitPlane(float2* points, float* heights, float* planeCoeffs)
{
    // Simple least squares plane fitting
    float sumX = 0, sumY = 0, sumZ = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumXZ = 0, sumYZ = 0;
    for (int i = 0; i < sizeof(points); i++)
    {
        float x = points[i].x;
        float y = points[i].y;
        float z = heights[i];
        sumX += x;
        sumY += y;
        sumZ += z;
        sumX2 += x * x;
        sumY2 += y * y;
        sumXY += x * y;
        sumXZ += x * z;
        sumYZ += y * z;
    }

    float det = sizeof(points) * (sumX2 * sumY2 - sumXY * sumXY) +
                sumX * (sumXY * sumY - sumX * sumY2) +
                sumY * (sumX * sumXY - sumX2 * sumY);
    float a = (sizeof(points) * (sumXZ * sumY2 - sumYZ * sumXY) +
               sumY * (sumYZ * sumX - sumXZ * sumY) +
               sumZ * (sumXY * sumY - sumX * sumY2)) / det;
    float b = (sizeof(points) * (sumX2 * sumYZ - sumXZ * sumXY) +
               sumX * (sumXZ * sumY - sumYZ * sumX) +
               sumZ * (sumX * sumXY - sumX2 * sumY)) / det;
    float c = (sumZ - a * sumX - b * sumY) / sizeof(points);

    // Convert to four-coefficient representation
    planeCoeffs[0] = a;
    planeCoeffs[1] = b;
    planeCoeffs[2] = -1.0f;
    planeCoeffs[3] = c;
}
