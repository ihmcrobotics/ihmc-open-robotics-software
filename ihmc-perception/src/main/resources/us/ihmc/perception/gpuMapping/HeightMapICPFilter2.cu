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
__global__ void findNearestNeighborsKernel(const float* local_map,
                                           const float* global_map,
                                           int* correspondences,
                                           float* distances,
                                           int local_size,
                                           int global_size)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx >= local_size) return;

    // Access local point (x, y, z) from flat array
    float local_x = local_map[idx * 3 + 0];
    float local_y = local_map[idx * 3 + 1];
    float local_z = local_map[idx * 3 + 2];

    float min_dist = 10000000;
    int min_idx = 0;

    for (int j = 0; j < global_size; ++j)
    {
        // Access global point (x, y, z) from flat array
        float global_x = global_map[j * 3 + 0];
        float global_y = global_map[j * 3 + 1];
        float global_z = global_map[j * 3 + 2];

        float dx = local_x - global_x;
        float dy = local_y - global_y;
        float dz = local_z - global_z;
        float dist = dx*dx + dy*dy + dz*dz;

        if (dist < min_dist)
        {
            min_dist = dist;
            min_idx = j;
        }
    }

    correspondences[idx] = min_idx;
    distances[idx] = sqrtf(min_dist);
}

extern "C"
__global__ void transformPointsKernel(float* points,
                                    const float* transform,
                                     int num_points)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

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