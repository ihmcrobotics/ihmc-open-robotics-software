#include "MathUtils.cuh"

extern "C"
#define DEPTH_INPUT_HEIGHT 0
#define DEPTH_INPUT_WIDTH 1
#define DEPTH_CX 2
#define DEPTH_CY 3
#define DEPTH_FX 4
#define DEPTH_FY 5
#define CELL_SIZE 6
#define CELLS_PER_AXIS 7

__device__ float3 back_project_perspective(int2 pos, float Z, const float *params)
{
    float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
    float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
    float3 point = make_float3(Z, -X, -Y);
    return point;
}

__device__ inline int flattenIndex(int ix, int iy, int iz, int dimX, int dimY, int dimZ)
{
   return ix + iy * dimX + iz * dimX * dimY;
}

extern "C"
__global__ void voxelMapKernel(unsigned short* depthImage, size_t pitchDepth,
                               int* voxelMap,   // 3D voxel array
                               float* sensorToWorldAlignedGroundTf,
                               float* params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    const int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);
    const int cellsPerAxis = static_cast<int>(params[CELLS_PER_AXIS]);

    if (xIndex >= depthWidth || yIndex >= depthHeight)
        return;

    // Coalesced read from depth image
    const unsigned short* rowPtr = (const unsigned short*)((const char*)depthImage + yIndex * pitchDepth);
    float depth = rowPtr[xIndex] * 0.001f; // Scale to meters

    // Early exit for invalid depth
    if (depth < 0.25f)
        return;

    float3 pointInSensorFrame = back_project_perspective(make_int2(xIndex, yIndex), depth, params);

    // Transform to world
    float3 pointInZUpWorldAlignedGroundFrame = transformPoint3D(pointInSensorFrame, sensorToWorldAlignedGroundTf);

    // Compute voxel indices
    // Because the center of the voxel grid is at the transform center, no need to subtract any origin
    float half = 0.5f * cellsPerAxis * params[CELL_SIZE];
    int ix = (int) floorf((pointInZUpWorldAlignedGroundFrame.x + half) / params[CELL_SIZE]);
    int iy = (int) floorf((pointInZUpWorldAlignedGroundFrame.y + half) / params[CELL_SIZE]);
    int iz = (int) floorf((pointInZUpWorldAlignedGroundFrame.z + half) / params[CELL_SIZE]);

    // Bounds check against the local map dimensions because we are scanning the entire depth image
    if (ix < 0 || ix >= cellsPerAxis || iy < 0 || iy >= cellsPerAxis || iz < 0 || iz >= cellsPerAxis)
        return;

    int idx = flattenIndex(ix, iy, iz, cellsPerAxis, cellsPerAxis, cellsPerAxis);

    // Atomic mark as occupied (1)
    atomicOr(&voxelMap[idx], 1);
}