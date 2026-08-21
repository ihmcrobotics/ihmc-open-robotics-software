#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define GLOBAL_CENTER_INDEX 2
#define LOCAL_CELLS_PER_AXIS 3
#define GLOBAL_CELLS_PER_AXIS 4
#define DEPTH_INPUT_HEIGHT 5
#define DEPTH_INPUT_WIDTH 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define DEPTH_FX 9
#define DEPTH_FY 10
#define MIN_HEIGHT_REGISTRATION 11
#define MAX_HEIGHT_REGISTRATION 12
#define MIN_CLAMP_HEIGHT 13
#define MAX_CLAMP_HEIGHT 14
#define KALMAN_FILTER_PREDICTION_NOISE 15
#define ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED 16
#define VARIANCE_PER_METER 17
#define VARIANCE_PER_TRANSLATION_SPEED 18
#define VARIANCE_PER_ROTATION_SPEED 19
#define GROUND_HEIGHT 20
#define MIN_DEPTH_TO_ACCEPT 21
#define ICP_OUTLIER_DISTANCE_THRESHOLD 22
#define ICP_VARIANCE_PER_METER_OF_CORRECTION 23

// Sentinel written to a global-map variance cell to mark it as holding no real data.
// Real variances are always non-negative, so this is unambiguous everywhere it's checked.
#define INVALID_CELL_VARIANCE -1.0f

__device__ float3 back_project_perspective(int2 pos, float Z, const float *params)
{
    float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
    float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
    float3 point = make_float3(Z, -X, -Y);
    return point;
}

/**
 * @brief Height map update KERNEL: One thread per depth pixel for peak optimization.
 * Reads in the depth for each pixel in the image.
 * Back projects the points into the Z-Up frame using transforms.
 * Atomically adds the points to the maps.
 */
extern "C"
__global__ void heightMapUpdateDataKernel(const unsigned short* __restrict__ depthImage, size_t pitchDepth,
                                          float* __restrict__ sumMap, size_t pitchSum,
                                          int* __restrict__ countMap, size_t pitchCount,
                                          float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                          float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                          const float* __restrict__ params,
                                          const float* __restrict__ sensorToZUpFrameTf,
                                          const float linearMotionMagnitude,
                                          const float angularMotionMagnitude)
{
    // Thread indices now correspond to pixel coordinates
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    const int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);
    const int cellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const float cellSize = params[CELL_SIZE];
    const int centerIndex = static_cast<int>(params[LOCAL_CENTER_INDEX]);
    const float varPerMeter = params[VARIANCE_PER_METER];
    const float varPerTranslationSpeed = params[VARIANCE_PER_TRANSLATION_SPEED];
    const float varPerRotationSpeed = params[VARIANCE_PER_ROTATION_SPEED];

    // Bounds check against depth image dimensions
    if (xIndex >= depthWidth || yIndex >= depthHeight)
        return;

    // Coalesced read from depth image
    const unsigned short* rowPtr = (const unsigned short*)((const char*)depthImage + yIndex * pitchDepth);
    float depth = rowPtr[xIndex] * 0.001f; // Scale to meters

    // Early exit for invalid depth
    if (depth < params[MIN_DEPTH_TO_ACCEPT])
        return;

    // Back-project and transform
    float3 queryPointInSensorFrame = back_project_perspective(make_int2(xIndex, yIndex), depth, params);
    float3 queryPointInZUpFrame = transformPoint3D(queryPointInSensorFrame, sensorToZUpFrameTf);

    // Get the correct cell index for the maps
    float2 xyCoords = make_float2(queryPointInZUpFrame.x, queryPointInZUpFrame.y);
    int2 cellIndex = coordinate_to_indices(xyCoords, make_float2(0.0f, 0.0f), cellSize, centerIndex);

    // Bounds check against the local map dimensions because we are scanning the entire depth image
    if (cellIndex.x < 0 || cellIndex.x >= cellsPerAxis || cellIndex.y < 0 || cellIndex.y >= cellsPerAxis)
        return;

    // Pointers to the target cell in each temporary map
    float* sumPtr = (float*)((char*)sumMap + cellIndex.y * pitchSum) + cellIndex.x;
    int* countPtr = (int*)((char*)countMap + cellIndex.y * pitchCount) + cellIndex.x;
    float* sumSqPtr = (float*)((char*)sumOfSquaresMap + cellIndex.y * pitchSumSq) + cellIndex.x;
    float* motionVarPtr = (float*)((char*)motionVarianceSumMap + cellIndex.y * pitchMotionVar) + cellIndex.x;

    // Atomically add this pixel's contribution
    atomicAdd(sumPtr, queryPointInZUpFrame.z);
    atomicAdd(countPtr, 1);
    atomicAdd(sumSqPtr, queryPointInZUpFrame.z * queryPointInZUpFrame.z);

    // Also calculate and add motion variance contribution
    float distance = sqrtf(queryPointInSensorFrame.x * queryPointInSensorFrame.x +
                           queryPointInSensorFrame.y * queryPointInSensorFrame.y +
                           queryPointInSensorFrame.z * queryPointInSensorFrame.z);
    float motionVarianceF = distance * varPerMeter + linearMotionMagnitude * varPerTranslationSpeed + angularMotionMagnitude * distance * varPerRotationSpeed;
    atomicAdd(motionVarPtr, motionVarianceF);
}

/**
 * @brief Compute Local Map KERNEL: The threads correspond to cell indices
 * Takes the maps that hold all the sums, and computes the mean, variance, and motion variance per cell
 */
extern "C"
__global__ void computeLocalMap(const float* __restrict__ sumMap, size_t pitchSum,
                                const int* __restrict__ countMap, size_t pitchCount,
                                const float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                const float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                float* __restrict__ localMeanMap, size_t pitchLocalMean,
                                float* __restrict__ localVarianceMap, size_t pitchLocalVariance,
                                float* __restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                const float* __restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int cellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);

    // Bounds check against local map dimensions
    if (xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
        return;

    const float* sumPtr = (const float*)((const char*)sumMap + yIndex * pitchSum) + xIndex;
    const int* countPtr = (const int*)((const char*)countMap + yIndex * pitchCount) + xIndex;
    const float* sumSqPtr = (const float*)((const char*)sumOfSquaresMap + yIndex * pitchSumSq) + xIndex;
    const float* motionVarPtr = (const float*)((const char*)motionVarianceSumMap + yIndex * pitchMotionVar) + xIndex;

    float count = *countPtr;
    float mean = 0.0f;
    float variance = 0.0f;
    float motionVariance = 0.0f;

    // Only compute if one or more points landed in this cell
    if (count > 0)
    {
        float sum = *sumPtr;
        mean = sum / count;
        motionVariance = *motionVarPtr / count;

        if (count > 1)
        {
            float sumSq = *sumSqPtr;
            // Unbiased sample variance (one-pass), a.k.a. Bessel's Correction:
            // (Σx² - (Σx)² / N) / (N - 1)
            variance = (sumSq - (sum * sum) / static_cast<float>(count)) / (static_cast<float>(count) - 1.0f);
            variance = fmaxf(0.0f, variance); // Clamp to zero to avoid negatives from float error
        }
    }

    // Write final results to the output maps
    float* meanHeight = (float*)((char*)localMeanMap + xIndex * pitchLocalMean) + yIndex;
    float* var = (float*)((char*)localVarianceMap + xIndex * pitchLocalVariance) + yIndex;
    float* motionVar = (float*)((char*)localMotionVarianceMap + xIndex * pitchLocalMotionVariance) + yIndex;

    *meanHeight = mean;
    *var = variance;
    *motionVar = motionVariance;
}

extern "C"
__global__ void translateHeightMapKernel(float *oldHeightMapMean, size_t pitchOldHeightMapMean,
                                         float *oldHeightMapVariance, size_t pitchOldHeightMapVariance,
                                         float *newMeanMap, size_t pitchNewMean,
                                         float *newVarianceMap, size_t pitchNewVariance,
                                         int shiftX, int shiftY, float *params, float defaultValue)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    if (x >= globalCellsPerAxis || y >= globalCellsPerAxis)
        return;

    int srcX = x + shiftX;
    int srcY = y + shiftY;

    float* newMeanRow = (float*)((char*)newMeanMap + x * pitchNewMean);
    float* newVarianceRow = (float*)((char*)newVarianceMap + x * pitchNewVariance);

    if (srcX >= 0 && srcX < globalCellsPerAxis && srcY >= 0 && srcY < globalCellsPerAxis)
    {
        float* oldMeanRow = (float*)((char*)oldHeightMapMean + srcX * pitchOldHeightMapMean);
        float* oldVarianceRow = (float *)((char*)oldHeightMapVariance + srcX * pitchOldHeightMapVariance);

        float oldVariance = oldVarianceRow[srcY];

        if (oldVariance < 0.0f)
        {
            // Source cell holds no real data, propagate that instead of manufacturing a variance for it
            newMeanRow[y] = defaultValue;
            newVarianceRow[y] = INVALID_CELL_VARIANCE;
        }
        else
        {
            newMeanRow[y] = oldMeanRow[srcY];
            // Add variance due to the translation
            newVarianceRow[y] = oldVariance + params[ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED];
        }
    }
    else
    {
        newMeanRow[y] = defaultValue;
        newVarianceRow[y] = INVALID_CELL_VARIANCE;
    }
}

/**
 * @brief ICP Correspondence KERNEL: One thread per local (new) cell.
 *
 * The state estimator can drift (mostly vertically, but also horizontally and in yaw about the
 * world Z axis) in ways that the translate kernel's integer cell shift doesn't capture, since that
 * shift only accounts for known/estimated motion. This kernel finds point correspondences between
 * the newest local data (treated as ground truth for this frame) and the existing global map
 * (which may have drifted), and accumulates the sums needed to solve a small-angle-linearized
 * Gauss-Newton update for the rigid transform (tx, ty, tz, yaw) that should be applied to the
 * OLD map to bring it back into alignment. It is meant to be called iteratively: each call takes
 * the current best estimate of that transform (totalTx/Ty/Tz/Yaw) and produces the sums for the
 * next incremental refinement, without modifying the global map itself.
 *
 * Accumulator layout (8 floats, atomically added into by every surviving correspondence):
 *   [0] N               : number of surviving correspondences
 *   [1] sum_rqx         : sum of the old point's world X, rotated by the current yaw estimate
 *   [2] sum_rqy         : sum of the old point's world Y, rotated by the current yaw estimate
 *   [3] sum_rq2         : sum of (rqx^2 + rqy^2)
 *   [4] sum_rx          : sum of the X residual (predicted old point minus new point)
 *   [5] sum_ry          : sum of the Y residual
 *   [6] sum_rz          : sum of the Z residual
 *   [7] sum_yawRhs      : sum of (rqy * rx - rqx * ry), the yaw right-hand-side term
 */
extern "C"
__global__ void icpCorrespondenceKernel(const float* __restrict__ localMeanMap, size_t pitchLocalMean,
                                        const float* __restrict__ globalMeanMap, size_t pitchGlobalMean,
                                        const float* __restrict__ globalVarianceMap, size_t pitchGlobalVariance,
                                        const float globalMapCenterX,
                                        const float globalMapCenterY,
                                        const float* __restrict__ groundToWorldTranslation,
                                        const float totalTx,
                                        const float totalTy,
                                        const float totalTz,
                                        const float totalYaw,
                                        const int searchRadiusCells,
                                        float* __restrict__ accumulator,
                                        const float* __restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);
    const float cellSize = params[CELL_SIZE];
    const int localCenterIndex = static_cast<int>(params[LOCAL_CENTER_INDEX]);
    const int globalCenterIndex = static_cast<int>(params[GLOBAL_CENTER_INDEX]);
    const float outlierDistanceThreshold = params[ICP_OUTLIER_DISTANCE_THRESHOLD];

    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    int2 localCell = make_int2(yIndex, xIndex);
    const float* localMeanRow = (const float*)((const char*)localMeanMap + localCell.x * pitchLocalMean);
    float localMeanF = localMeanRow[localCell.y];

    // No point landed in this local cell, nothing to correspond
    if (localMeanF == 0.0f)
        return;

    float2 localCoordinate = indices_to_coordinate(localCell, make_float2(0.0f, 0.0f), cellSize, localCenterIndex);
    float3 pointInLocalFrame = make_float3(localCoordinate.x, localCoordinate.y, 0.0f);
    float3 pointInGroundToWorld = transformPoint3D(pointInLocalFrame, groundToWorldTranslation);

    // The newest sensor data is treated as the fixed reference this frame
    float3 newPoint = make_float3(pointInGroundToWorld.x, pointInGroundToWorld.y, localMeanF);

    float cosYaw = cosf(totalYaw);
    float sinYaw = sinf(totalYaw);

    // Where, in the OLD map's raw (uncorrected) indexing, the corresponding cell should be,
    // given the current correction estimate: the inverse of the current estimate applied to newPoint
    float invX = newPoint.x - totalTx;
    float invY = newPoint.y - totalTy;
    float searchCenterX = cosYaw * invX + sinYaw * invY;
    float searchCenterY = -sinYaw * invX + cosYaw * invY;

    int2 searchCenterCell = coordinate_to_indices(make_float2(searchCenterX, searchCenterY),
                                                  make_float2(globalMapCenterX, globalMapCenterY),
                                                  cellSize,
                                                  globalCenterIndex);

    bool foundMatch = false;
    float bestDistanceSquared = 0.0f;
    float bestRqx = 0.0f;
    float bestRqy = 0.0f;
    float bestQz = 0.0f;

    for (int dcx = -searchRadiusCells; dcx <= searchRadiusCells; ++dcx)
    {
        for (int dcy = -searchRadiusCells; dcy <= searchRadiusCells; ++dcy)
        {
            int candidateX = searchCenterCell.x + dcx;
            int candidateY = searchCenterCell.y + dcy;

            if (candidateX < 0 || candidateX >= globalCellsPerAxis || candidateY < 0 || candidateY >= globalCellsPerAxis)
                continue;

            const float* varianceRow = (const float*)((const char*)globalVarianceMap + candidateX * pitchGlobalVariance);
            if (varianceRow[candidateY] < 0.0f)
                continue; // no real data at this candidate

            const float* meanRow = (const float*)((const char*)globalMeanMap + candidateX * pitchGlobalMean);
            float qz = meanRow[candidateY];

            float2 qWorld = indices_to_coordinate(make_int2(candidateX, candidateY),
                                                  make_float2(globalMapCenterX, globalMapCenterY),
                                                  cellSize,
                                                  globalCenterIndex);

            // Predict where this old cell lands under the current correction estimate
            float rqx = cosYaw * qWorld.x - sinYaw * qWorld.y;
            float rqy = sinYaw * qWorld.x + cosYaw * qWorld.y;
            float predictedX = rqx + totalTx;
            float predictedY = rqy + totalTy;
            float predictedZ = qz + totalTz;

            float dx = predictedX - newPoint.x;
            float dy = predictedY - newPoint.y;
            float dz = predictedZ - newPoint.z;
            float distSq = dx * dx + dy * dy + dz * dz;

            if (!foundMatch || distSq < bestDistanceSquared)
            {
                foundMatch = true;
                bestDistanceSquared = distSq;
                bestRqx = rqx;
                bestRqy = rqy;
                bestQz = qz;
            }
        }
    }

    if (!foundMatch || sqrtf(bestDistanceSquared) > outlierDistanceThreshold)
        return;

    float predictedX = bestRqx + totalTx;
    float predictedY = bestRqy + totalTy;
    float predictedZ = bestQz + totalTz;

    float rx = predictedX - newPoint.x;
    float ry = predictedY - newPoint.y;
    float rz = predictedZ - newPoint.z;
    float yawRhsTerm = bestRqy * rx - bestRqx * ry;

    atomicAdd(&accumulator[0], 1.0f);
    atomicAdd(&accumulator[1], bestRqx);
    atomicAdd(&accumulator[2], bestRqy);
    atomicAdd(&accumulator[3], bestRqx * bestRqx + bestRqy * bestRqy);
    atomicAdd(&accumulator[4], rx);
    atomicAdd(&accumulator[5], ry);
    atomicAdd(&accumulator[6], rz);
    atomicAdd(&accumulator[7], yawRhsTerm);
}

/**
 * @brief ICP Apply Correction KERNEL: One thread per global cell.
 *
 * Applies the final, converged (tx, ty, tz, yaw) correction (accumulated on the CPU from repeated
 * calls to {@code icpCorrespondenceKernel}) to the global map, the same double-buffered
 * old-map/new-map pattern used by {@code translateHeightMapKernel}. For each destination cell, the
 * inverse of the correction is used to find which old (uncorrected) cell its data should come from,
 * snapping to the nearest cell rather than interpolating. The variance is bumped in proportion to
 * how far that cell's data actually moved, so more heavily corrected cells are trusted less.
 */
extern "C"
__global__ void icpApplyCorrectionKernel(float* oldHeightMapMean, size_t pitchOldHeightMapMean,
                                         float* oldHeightMapVariance, size_t pitchOldHeightMapVariance,
                                         float* newMeanMap, size_t pitchNewMean,
                                         float* newVarianceMap, size_t pitchNewVariance,
                                         const float globalMapCenterX,
                                         const float globalMapCenterY,
                                         const float totalTx,
                                         const float totalTy,
                                         const float totalTz,
                                         const float totalYaw,
                                         float* params,
                                         float defaultValue)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);
    float cellSize = params[CELL_SIZE];
    int globalCenterIndex = static_cast<int>(params[GLOBAL_CENTER_INDEX]);
    float variancePerMeter = params[ICP_VARIANCE_PER_METER_OF_CORRECTION];

    if (x >= globalCellsPerAxis || y >= globalCellsPerAxis)
        return;

    float2 destinationWorld = indices_to_coordinate(make_int2(x, y),
                                                    make_float2(globalMapCenterX, globalMapCenterY),
                                                    cellSize,
                                                    globalCenterIndex);

    float cosYaw = cosf(totalYaw);
    float sinYaw = sinf(totalYaw);

    // Invert the correction transform to find where this destination cell's data came from
    float shiftedX = destinationWorld.x - totalTx;
    float shiftedY = destinationWorld.y - totalTy;
    float sourceWorldX = cosYaw * shiftedX + sinYaw * shiftedY;
    float sourceWorldY = -sinYaw * shiftedX + cosYaw * shiftedY;

    int2 sourceCell = coordinate_to_indices(make_float2(sourceWorldX, sourceWorldY),
                                            make_float2(globalMapCenterX, globalMapCenterY),
                                            cellSize,
                                            globalCenterIndex);

    float* newMeanRow = (float*)((char*)newMeanMap + x * pitchNewMean);
    float* newVarianceRow = (float*)((char*)newVarianceMap + x * pitchNewVariance);

    if (sourceCell.x < 0 || sourceCell.x >= globalCellsPerAxis || sourceCell.y < 0 || sourceCell.y >= globalCellsPerAxis)
    {
        newMeanRow[y] = defaultValue;
        newVarianceRow[y] = INVALID_CELL_VARIANCE;
        return;
    }

    float* oldVarianceRow = (float*)((char*)oldHeightMapVariance + sourceCell.x * pitchOldHeightMapVariance);
    float sourceVariance = oldVarianceRow[sourceCell.y];

    if (sourceVariance < 0.0f)
    {
        newMeanRow[y] = defaultValue;
        newVarianceRow[y] = INVALID_CELL_VARIANCE;
        return;
    }

    float* oldMeanRow = (float*)((char*)oldHeightMapMean + sourceCell.x * pitchOldHeightMapMean);
    float sourceMean = oldMeanRow[sourceCell.y];

    // The physical displacement this cell's data underwent due to the correction
    float dx = destinationWorld.x - sourceWorldX;
    float dy = destinationWorld.y - sourceWorldY;
    float displacement = sqrtf(dx * dx + dy * dy + totalTz * totalTz);

    newMeanRow[y] = sourceMean + totalTz;
    newVarianceRow[y] = sourceVariance + variancePerMeter * displacement;
}

extern "C"
__global__ void heightMapRegistrationKernel(const float *__restrict__ localMeanMap, size_t pitchLocalMean,
                                            const float *__restrict__ localVarianceMap, size_t pitchLocalVariance,
                                            const float *__restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                            float *__restrict__ globalMeanMap, size_t pitchGlobalMean,
                                            float *__restrict__ globalVarianceMap, size_t pitchGlobalVariance,
                                            const float globalMapCenterX,
											const float globalMapCenterY,
                                            const float *__restrict__ groundToWorldTranslation,
                                            const float *__restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for local indices
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    // Doing (y, x) allows for coalesced memory access when going to global memory
    int2 localCell = make_int2(yIndex, xIndex);
    float *localMean = (float *)((char *)localMeanMap + localCell.x * pitchLocalMean) + localCell.y;
    // Global memory access is asychronious and takes a long time, tell the bus to go grab some memory
    float localMeanF = *localMean;

    // While the global memory is being fetched, convert the local cell into the global cell so we can register the data
    float2 localCoordinate = indices_to_coordinate(localCell, make_float2(0.0f, 0.0f), params[CELL_SIZE], params[LOCAL_CENTER_INDEX]);
    float3 pointInLocalFrame = make_float3(localCoordinate.x, localCoordinate.y, 0.0f);
    float3 pointInGlobalFrame = transformPoint3D(pointInLocalFrame, groundToWorldTranslation);
    int2 globalCell = coordinate_to_indices(make_float2(pointInGlobalFrame.x, pointInGlobalFrame.y), make_float2(globalMapCenterX, globalMapCenterY), params[CELL_SIZE], params[GLOBAL_CENTER_INDEX]);

    if (globalCell.x < 0 || globalCell.x >= globalCellsPerAxis || globalCell.y < 0 || globalCell.y >= globalCellsPerAxis)
        return;

    // The work done so far by the kernel is very small, so the local mean global memory access hasn't finished yet.
    // Setup the correct address for the data we want to use later
    float *localVariance = (float *)((char *)localVarianceMap + localCell.x * pitchLocalVariance) + localCell.y;
    float *localMotionVariance = (float *)((char *)localMotionVarianceMap + localCell.x * pitchLocalMotionVariance) + localCell.y;
    float *globalMean = (float *)((char *)globalMeanMap + globalCell.x * pitchGlobalMean) + globalCell.y;
    float *globalVariance = (float *)((char *)globalVarianceMap + globalCell.x * pitchGlobalVariance) + globalCell.y;

    // After trying to do as much work as possible in parallel to the global memory access. We ran out of stuff to do.
    // Check the result of global memory for invalid data, and return if not valid
    if (localMeanF == 0)
        return;

    // Access the rest of the global memory needed for the filtering, there is no other work to do
    float localVarianceF = *localVariance;
    float globalMeanF = *globalMean;
    float globalVarianceF = *globalVariance;

    // If we have no real data, we don't apply the kalman filter, just take the new real data
    if (globalVarianceF <= 0.0f)
    {
        *globalMean = localMeanF;
        *globalVariance = localVarianceF;
    }
    else
    {
        // Too many global memory access's at once seems to overwhelm the memory bus, do the last global memory access here to space things out a little
        float localMotionVarianceF = *localMotionVariance;

        // Predict step of the kalman filter
        float predictedMean = globalMeanF;
        float predictedVariance = globalVarianceF + params[KALMAN_FILTER_PREDICTION_NOISE];

        // Update step of the kalman filter
        float kalmanGain = predictedVariance / (predictedVariance + localVarianceF + localMotionVarianceF);
        float updatedMean = predictedMean + kalmanGain * (localMeanF - predictedMean);
        float updatedVariance = (1.0f - kalmanGain) * predictedVariance;

        *globalMean = updatedMean;
        *globalVariance = updatedVariance;
    }
}

extern "C"
__global__ void heightMapEmptyRegistrationKernel(float *localMap, size_t pitchLocal,
                                                 float *globalMap, size_t pitchGlobal,
                                                 float *groundToWorldTranslation,
                                                 float *params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Compute global map size
    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    int2 localIndex = make_int2(xIndex, yIndex);
    float2 localCoordinate = indices_to_coordinate(localIndex, make_float2(0.0f, 0.0f), params[CELL_SIZE], params[LOCAL_CENTER_INDEX]);
    float3 queryPointInLocal = make_float3(localCoordinate.x, localCoordinate.y, 0.0f);
    float3 cellInGlobal = transformPoint3D(queryPointInLocal, groundToWorldTranslation);
    int2 globalIndex = coordinate_to_indices(make_float2(cellInGlobal.x, cellInGlobal.y), make_float2(groundToWorldTranslation[3], groundToWorldTranslation[7]), params[CELL_SIZE], params[GLOBAL_CENTER_INDEX]);

    if (globalIndex.x < 0 || globalIndex.x >= globalCellsPerAxis || globalIndex.y < 0 || globalIndex.y >= globalCellsPerAxis)
        return;

    float *localHeight = (float *)((char *)localMap + localIndex.x * pitchLocal) + localIndex.y;

    // This is a way of checking if we have any read data in the spot, its very unlikely that any real data is 0.0
    if (*localHeight == 0.0f)
        return;

    float *globalHeight = (float *)((char *)globalMap + globalIndex.x * pitchGlobal) + globalIndex.y;
    *globalHeight = *localHeight;
}

extern "C"
/**
    @brief Compute Plan Offset KERNEL: This kernel is not very intuitive. For background we've got a status message
    being published that gives us an offset in Z in meters that the robot thinks its drifted by.
    Our goal is to update the height map accordingly. However, it doesn't make sense to update any values that are
    in the live view of the camera, because: a) they will just get overwritten the next image, and b) that data doesn't drift
    because its live data. So we pass in this matrix of values to skip because those values are outside the live view
    of the camera.
*/
__global__ void planOffsetKernel(float *matrixToModify, size_t pitchMatrixToModify,
                                 float *matrixValuesToSkip, size_t pitchMatrixValuesToSkip,
                                 float offsetInZ, float zeroValueForEmptySpaces, float *params)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    if (indexX >= globalCellsPerAxis || indexY >= globalCellsPerAxis)
        return;

    float *skipRow = (float *)((char *)matrixValuesToSkip + indexX * pitchMatrixValuesToSkip);
    // This skips the cells that have real data in them coming from the values to skip
    if (skipRow[indexY] != zeroValueForEmptySpaces)
        return;

    float *matrixRow = (float *)((char *)matrixToModify + indexX * pitchMatrixToModify);
    matrixRow[indexY] += offsetInZ;
}