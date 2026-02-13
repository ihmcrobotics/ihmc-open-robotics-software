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
__global__ void v2(float *__restrict__ localMap, size_t pitchLocal,
                   float *__restrict__ globalMap, size_t pitchGlobal,
                   int *__restrict__ localKeys,
                   int *__restrict__ globalKeys,
                   float* distances,
                   int* validCounter,
                   const float *__restrict__ localToGlobalTransform,
                   const float globalMapCenterX,
                   const float globalMapCenterY,
                   const float *__restrict__ params)
{
    int lx = blockIdx.x * blockDim.x + threadIdx.x;
    int ly = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for local indices
    if (lx >= localCellsPerAxis || ly >= localCellsPerAxis)
        return;

    // Doing (y, x) allows for coalesced memory access when going to global memory
    int2 localCell = make_int2(ly, lx);
    float *localMean = (float *)((char *)localMap + localCell.x * pitchLocal) + localCell.y;
    // Global memory access is asynchronous and takes a long time, tell the bus to go grab some memory
    float localMeanF = *localMean;

    // While the global memory is being fetched, convert the local cell into the global cell so we can register the data
    float2 localCoordinate = indices_to_coordinate(localCell, make_float2(0.0f, 0.0f), params[CELL_SIZE], params[LOCAL_CENTER_INDEX]);
    float3 pointInLocalFrame = make_float3(localCoordinate.x, localCoordinate.y, localMeanF);
    float3 pointInGlobalFrame = transformPoint3D(pointInLocalFrame, localToGlobalTransform);
    int2 globalCell = coordinate_to_indices(make_float2(pointInGlobalFrame.x, pointInGlobalFrame.y), make_float2(globalMapCenterX, globalMapCenterY), params[CELL_SIZE], params[GLOBAL_CENTER_INDEX]);

    if (globalCell.x < 0 || globalCell.x >= globalCellsPerAxis || globalCell.y < 0 || globalCell.y >= globalCellsPerAxis)
        return;

    // After trying to do as much work as possible in parallel to the global memory access. We ran out of stuff to do.
    // Check the result of global memory for invalid data, and return if not valid
    if (localMeanF == 0)
        return;

    int searchRadius = params[SEARCH_RADIUS];
    int best_global_key = -1;
    float minimum_distance = 1e10f;

    for (int dx = -searchRadius; dx <= searchRadius; dx++)
    {
        for (int dy = -searchRadius; dy <= searchRadius; dy++)
        {
            int searchX = globalCell.x + dx;
            int searchY = globalCell.y + dy;

            if (searchX >= 0 && searchX < params[GLOBAL_CELLS_PER_AXIS] &&
                searchY >= 0 && searchY < params[GLOBAL_CELLS_PER_AXIS])
            {
                float* globalRow = (float*)((char*)globalMap + searchY * pitchGlobal);
                float globalHeight = globalRow[searchX];
                float2 globalCellCoords = indices_to_coordinate(
                    make_int2(searchX, searchY),
                    make_float2(localToGlobalTransform[3], localToGlobalTransform[7]),
                    params[CELL_SIZE],
                    params[GLOBAL_CENTER_INDEX]);

                float3 candidatePoint = make_float3(globalCellCoords.x, globalCellCoords.y, globalHeight);

                float deltaX = pointInGlobalFrame.x - candidatePoint.x;
                float deltaY = pointInGlobalFrame.y - candidatePoint.y;
                float deltaZ = pointInGlobalFrame.z - candidatePoint.z;
                float distance = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;

                if (distance < minimum_distance)
                {
                    // Keep track of the shortest distance from the local point to the global point
                    // Also note the index of this
                    minimum_distance = distance;
                    int globalCells = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);
                    best_global_key = searchX * globalCells + searchY; // Using your indicesToKey logic
                }
            }
        }
    }

    if (best_global_key != -1)
    {
        // atomicAdd returns the OLD value and increments the memory by 1
        int writeIndex = atomicAdd(validCounter, 1);

        int localCells = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
        localKeys[writeIndex] = lx * localCells + ly; // indicesToKey logic
        globalKeys[writeIndex] = best_global_key;
        distances[writeIndex] = sqrtf(minimum_distance);
    }
}

extern "C"
__global__ void findNearestNeighborsKernel(const float* __restrict__ local_map,
                                           const float* __restrict__ global_map,
                                           int* correspondences,
                                           float* distances,
                                           int local_size,
                                           int global_size)
{
    // Use a fixed tile size for shared memory
    const int TILE_SIZE = 256;
    __shared__ float shared_global[TILE_SIZE * 3];

    int index = blockIdx.x * blockDim.x + threadIdx.x;

    float local_x, local_y, local_z;
    if (index < local_size) {
        local_x = local_map[index * 3 + 0];
        local_y = local_map[index * 3 + 1];
        local_z = local_map[index * 3 + 2];
    }

    float min_dist_sq = 1e20f; // Use a large float constant
    int min_idx = 0;

    // Loop through global map in tiles
    for (int tile_start = 0; tile_start < global_size; tile_start += TILE_SIZE) {

        // Coalesced load into shared memory
        int threads_to_load = min(TILE_SIZE, global_size - tile_start);
        if (threadIdx.x < threads_to_load) {
            int g_idx = (tile_start + threadIdx.x) * 3;
            shared_global[threadIdx.x * 3 + 0] = global_map[g_idx + 0];
            shared_global[threadIdx.x * 3 + 1] = global_map[g_idx + 1];
            shared_global[threadIdx.x * 3 + 2] = global_map[g_idx + 2];
        }

        // Wait for all threads to finish loading the tile
        __syncthreads();

        // If this thread represents a valid local point, compare against the tile
        if (index < local_size) {
            for (int i = 0; i < threads_to_load; ++i) {
                float dx = local_x - shared_global[i * 3 + 0];
                float dy = local_y - shared_global[i * 3 + 1];
                float dz = local_z - shared_global[i * 3 + 2];
                float d_sq = dx * dx + dy * dy + dz * dz;

                if (d_sq < min_dist_sq) {
                    min_dist_sq = d_sq;
                    min_idx = tile_start + i;
                }
            }
        }

        // Ensure all threads are done with the tile before loading the next one
        __syncthreads();
    }

    if (index < local_size) {
        correspondences[index] = min_idx;
        distances[index] = sqrtf(min_dist_sq);
    }
}

extern "C"
__global__ void transformPointsKernel(float* points,
                                      const float* transform,
                                      int num_points)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points)
        return;

    int base = idx * 3;
    float x = points[base + 0];
    float y = points[base + 1];
    float z = points[base + 2];

    // transform[0-3]   is row 0 (R00, R01, R02, Tx)
    // transform[4-7]   is row 1 (R10, R11, R12, Ty)
    // transform[8-11]  is row 2 (R20, R21, R22, Tz)

    float new_x = transform[0] * x + transform[1] * y + transform[2] * z + transform[3];
    float new_y = transform[4] * x + transform[5] * y + transform[6] * z + transform[7];
    float new_z = transform[8] * x + transform[9] * y + transform[10] * z + transform[11];

    points[base + 0] = new_x;
    points[base + 1] = new_y;
    points[base + 2] = new_z;
}