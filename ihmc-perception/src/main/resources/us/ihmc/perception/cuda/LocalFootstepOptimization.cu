#define GRID_RESOLUTION_YAW 0.0872664626f // 5 degrees in radians
#define POSITION_W 10.0f
#define YAW_W 1.0f
#define PLANARITY_W 100.0f
#define HEIGHT_W 20.0f
#define MAX_HEIGHT_VARIANCE 0.05f
#define CONTINUOUS_SURFACE_TOLERANCE 0.02f
#define MAX_SLOPE_ANGLE 0.872665f // approx 50 degrees in radians
#define DISCONTINUITY_PENALTY 50.0f
#define HEIGHT_CONSTRAINT_PENALTY 5.0f
#define SLOPE_CONSTRAINT_PENALTY 10.0f

// CUDA kernel to find the minimum value and its index
extern "C" __global__ void findBlockMinima(float* costs, int size, float* blockMinValues, int* blockMinIndices)
{
    // Shared memory for block-level reduction
    __shared__ float sharedMinValues[512];
    __shared__ int sharedMinIndices[512];

    int tid = threadIdx.x;                  // Thread ID within the block
    int gid = blockIdx.x * blockDim.x + tid; // Global thread ID

    // Initialize shared memory with sentinel values
    if (gid < size)
    {
        sharedMinValues[tid] = costs[gid];
        sharedMinIndices[tid] = gid;
    }
    else
    {
        sharedMinValues[tid] = 100000; // Sentinel value for out-of-bounds threads
        sharedMinIndices[tid] = -1;
    }
    __syncthreads();

    // Perform parallel reduction within the block to find the local minimum
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1)
    {
        if (tid < stride && tid + stride < size)
        {
            if (sharedMinValues[tid + stride] < sharedMinValues[tid])
            {
                sharedMinValues[tid] = sharedMinValues[tid + stride];
                sharedMinIndices[tid] = sharedMinIndices[tid + stride];
            }
        }
        __syncthreads();
    }

    // Block leader writes the result to global memory
    if (tid == 0)
    {
        blockMinValues[blockIdx.x] = sharedMinValues[0];
        blockMinIndices[blockIdx.x] = sharedMinIndices[0];
    }
}

extern "C" __global__ void findGlobalMinimum(float* blockMinCosts, int* blockMinIndices, int numBlocks, float* globalMinCost, int* globalMinIndex)
{
    __shared__ float sharedMinCosts[512];
    __shared__ int sharedMinIndices[512];

    int tid = threadIdx.x;

    // Initialize shared memory
    if (tid < numBlocks)
    {
        sharedMinCosts[tid] = blockMinCosts[tid];
        sharedMinIndices[tid] = blockMinIndices[tid];
    }
    else
    {
        sharedMinCosts[tid] = 100000;
        sharedMinIndices[tid] = -1;
    }
    __syncthreads();

    // Perform reduction
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1)
    {
        if (tid < stride && tid + stride < numBlocks)
        {
            if (sharedMinCosts[tid + stride] < sharedMinCosts[tid])
            {
                sharedMinCosts[tid] = sharedMinCosts[tid + stride];
                sharedMinIndices[tid] = sharedMinIndices[tid + stride];
            }
        }
        __syncthreads();
    }

    // Write global result
    if (tid == 0)
    {
        globalMinCost[0] = sharedMinCosts[0];
        globalMinIndex[0] = sharedMinIndices[0];
    }
}

__device__ int indicesToKey(int xIndex, int yIndex, int centerIndex)
{
    return  xIndex + yIndex * (2 * centerIndex + 1);
}

__device__ int coordinateToIndex(float coordinate, float gridCenter,
                                float resolution, int centerIndex)
{
    float offset = (coordinate - gridCenter) / resolution;
    return static_cast<int>(round(offset)) + centerIndex;
}

__device__ int coordinateToKey(float x, float y, float xCenter, float yCenter,
                              float resolution, int centerIndex)
{
    int xIndex = coordinateToIndex(x, xCenter, resolution, centerIndex);
    int yIndex = coordinateToIndex(y, yCenter, resolution, centerIndex);
    return indicesToKey(xIndex, yIndex, centerIndex);
}

// -------------------------------------------------------------------------
// Returns the height at world coordinates (x,y) given a height map that is
// stored as a contiguous float array
__device__ float getHeightAt(float x, float y, const float* heightMapData, float* heightMapCenter, int heightMapCenterIdx, float heightMapResolution)
{
    int key = coordinateToKey(x, y, heightMapCenter[0], heightMapCenter[1], heightMapResolution, heightMapCenterIdx);
    return (float)heightMapData[key];
}

__device__ void sampleFootCorners(
    float x, float y, float yaw,
    float* heightMapData,
    float* heightMapCenter,
    int heightMapCenterIdx,
    float heightMapResolution,
    float footLength,
    float footWidth,
    float2* cornersXYSubset,
    float* heightsSubset,
    float* heights,
    bool* isSurfaceDiscontinuous)
{
    // Define foot corners
    float fx = footLength; // keep some room for not bumping into edges
    float fy = footWidth;
    float2 corners[5] = {
        {-fx / 2, -fy / 2},
        {fx / 2, -fy / 2},
        {fx / 2, fy / 2},
        {-fx / 2, fy / 2},
        {0, 0}
    };

    float2 cornersXY[5];

    // Sample corner positions and heights
    for (int i = 0; i < 5; i++)
    {
        float sampleX = x + corners[i].x * cosf(yaw) - corners[i].y * sinf(yaw);
        float sampleY = y + corners[i].x * sinf(yaw) + corners[i].y * cosf(yaw);
        cornersXY[i].x = sampleX;
        cornersXY[i].y = sampleY;

        heights[i] = getHeightAt(sampleX, sampleY, heightMapData, heightMapCenter, heightMapCenterIdx, heightMapResolution);
    }

    // Find mean height
    float meanHeight = 0.0f;
    for (int i = 0; i < 5; i++)
    {
        meanHeight += heights[i];
    }
    meanHeight /= 5.0f;

    // Find corner with largest height difference to mean
    int maxDiffIndex = 0;
    float maxDiff = fabsf(heights[0] - meanHeight);
    for (int i = 1; i < 5; i++)
    {
        float diff = fabsf(heights[i] - meanHeight);
        if (diff > maxDiff)
        {
            maxDiff = diff;
            maxDiffIndex = i;
        }
    }

    // Remove the worst corner
    int subsetIndex = 0;
    for (int i = 0; i < 5; i++)
    {
        if (i != maxDiffIndex)
        {
            cornersXYSubset[subsetIndex] = cornersXY[i];
            heightsSubset[subsetIndex] = heights[i];
            subsetIndex++;
        }
    }

    // Check continuity of surface
    float frontHeightAvg = (heights[0] + heights[3]) / 2.0f;
    float backHeightAvg = (heights[1] + heights[2]) / 2.0f;

    *isSurfaceDiscontinuous =
        fabsf(heights[4] - (frontHeightAvg + backHeightAvg) / 2.0f) > CONTINUOUS_SURFACE_TOLERANCE;
}

__device__ float* fitPlane(float2* points, float* heights)
{
    // Allocate memory for plane coefficients
    static float planeCoeffs[4];
    // Simple least squares plane fitting
    float sumX = 0, sumY = 0, sumZ = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumXZ = 0, sumYZ = 0;
    for (int i = 0; i < 4; i++)
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

    float det = 4 * (sumX2 * sumY2 - sumXY * sumXY) +
                sumX * (sumXY * sumY - sumX * sumY2) +
                sumY * (sumX * sumXY - sumX2 * sumY);
    float a = (4 * (sumXZ * sumY2 - sumYZ * sumXY) +
               sumY * (sumYZ * sumX - sumXZ * sumY) +
               sumZ * (sumXY * sumY - sumX * sumY2)) / det;
    float b = (4 * (sumX2 * sumYZ - sumXZ * sumXY) +
               sumX * (sumXZ * sumY - sumYZ * sumX) +
               sumZ * (sumX * sumXY - sumX2 * sumY)) / det;
    float c = (sumZ - a * sumX - b * sumY) / 4;

    // Convert to four-coefficient representation
    planeCoeffs[0] = a;
    planeCoeffs[1] = b;
    planeCoeffs[2] = -1.0f;
    planeCoeffs[3] = c;

    return planeCoeffs;
}

// -------------------------------------------------------------------------
// Computes the planarity cost for a candidate footstep pose. In this function
// we sample 5 points (the four corners of the foot and its center), then if the
// surface appears discontinuous we return a heavy penalty; otherwise, we discard
// the worst (most deviant) sample, fit a plane to remaining points and compute rmse
// height deviation and inclination
__device__ float computePlanarityCost(
    float x, float y, float yaw,
    float* heightMapData,
    float* heightMapCenter,
    int heightMapCenterIdx,
    float heightMapResolution,
    float footLength,
    float footWidth)
{
    // Sample foot corners and check for surface discontinuity
    float2 cornersXYSubset[4];
    float heightsSubset[4];
    float heights[5];
    bool isSurfaceDiscontinuous;

    sampleFootCorners(
        x, y, yaw,
        heightMapData, heightMapCenter, heightMapCenterIdx, heightMapResolution,
        footLength, footWidth,
        cornersXYSubset, heightsSubset, heights, &isSurfaceDiscontinuous);

    if (isSurfaceDiscontinuous)
    {
        // Calculate height differences relative to the center point
        float diff1 = fabsf(heights[0] - heights[4]); // Corner 1 - Center
        float diff2 = fabsf(heights[1] - heights[4]); // Corner 2 - Center
        float diff3 = fabsf(heights[2] - heights[4]); // Corner 3 - Center
        float diff4 = fabsf(heights[3] - heights[4]); // Corner 4 - Center

        return DISCONTINUITY_PENALTY * (diff1 + diff2 + diff3 + diff4);
    }
    else
    {
        // Fit a plane to the remaining points
        float* planeCoeffs = fitPlane(cornersXYSubset, heightsSubset);

        // Calculate the sum of squared distances from each point to the fitted plane
        float sumDistances = 0.0f;
        float maxVariance = 0.0f;
        float normalizer = sqrtf(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + 1); // Since C = -1
        for (int i = 0; i < 4; i++)
        {
            float distance = fabsf(
                planeCoeffs[0] * cornersXYSubset[i].x +
                planeCoeffs[1] * cornersXYSubset[i].y -
                heightsSubset[i] + planeCoeffs[3]) / normalizer;

            sumDistances += distance;
            maxVariance = fmaxf(maxVariance, distance);
        }
        float meanError = sumDistances / 4;

        // Calculate the maximum slope of the fitted plane
        float maxSlope = atanf(sqrtf(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1]));
        // Add penalties if constraints are violated
        float penalties = 0.0f;
        if (maxVariance > MAX_HEIGHT_VARIANCE)
        {
            penalties += HEIGHT_CONSTRAINT_PENALTY;
        }
        if (maxSlope > MAX_SLOPE_ANGLE)
        {
            penalties += SLOPE_CONSTRAINT_PENALTY;
        }
        return meanError + penalties;
    }
}

__device__ float computeCost(float x, float y, float yaw, float* initialPose, float footLength, float footWidth, float* heightMapData, float* heightMapCenter, int heightMapCenterIdx, float heightMapResolution)
{
    float positionCost = POSITION_W * (fabsf(x - initialPose[0]) + fabsf(y - initialPose[1]));
    float yawCost = YAW_W * fabsf(yaw - initialPose[3]);
    float planarityCost = PLANARITY_W * computePlanarityCost(x, y, yaw, heightMapData, heightMapCenter, heightMapCenterIdx, heightMapResolution, footLength, footWidth);

    float z = getHeightAt(x, y, heightMapData, heightMapCenter, heightMapCenterIdx, heightMapResolution);
    float zDistancePenalty = HEIGHT_W * fabsf(z - initialPose[2]);

    return positionCost + yawCost + planarityCost + zDistancePenalty;
}

extern "C" __global__ void optimizeFootstep(float* heightMapData,
                                            float* heightMapCenter,
                                            int heightMapCenterIdx,
                                            float heightMapResolution,
                                            float* initialPose,
                                            float footLength,
                                            float footWidth,
                                            float searchRadius,
                                            float searchYawLimit,
                                            int stepsXY,
                                            int stepsYaw,
                                            float* costs,
                                            float* solutions)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int totalThreads = stepsXY * stepsXY * stepsYaw;

    if (idx < totalThreads)
    {
        // Divides the thread index by the total number of steps in the YZ plane
        int i = idx / (stepsXY * stepsYaw);
        // Divide idx by stepsYaw to get the position within the XY plane,
        // then uses the modulo operator to find its position within the Y direction of that plane.
        int j = (idx / stepsYaw) % stepsXY;
        // Use the modulo operator to find the remainder when divided by stepsYaw
        int k = idx % stepsYaw;

        float x = initialPose[0] - searchRadius + i * heightMapResolution;
        float y = initialPose[1] - searchRadius + j * heightMapResolution;
        float yaw = initialPose[3] - searchYawLimit + k * GRID_RESOLUTION_YAW;

        float cost = computeCost(x, y, yaw, initialPose, footLength, footWidth, heightMapData, heightMapCenter, heightMapCenterIdx, heightMapResolution);
        // Write the computed cost to the global memory array
        costs[idx] = cost;

        // Write the solution (x, y, yaw) to the global memory array
        solutions[idx * 3] = x;
        solutions[idx * 3 + 1] = y;
        solutions[idx * 3 + 2] = yaw;
    }
}
