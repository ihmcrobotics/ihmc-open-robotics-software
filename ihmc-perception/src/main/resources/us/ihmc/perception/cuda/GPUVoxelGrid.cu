#include "MathUtils.cuh"

/**
 * GPU log-odds voxel grid for persistent 3-D occupancy mapping.
 *
 * Log-odds stored as fixed-point integers (scale = 1000 units per 1.0 log-odds).
 * Grid layout: flat array, index = ix*NY*NZ + iy*NZ + iz (X major, Z minor).
 */

#define LOG_ODDS_HIT        847
#define LOG_ODDS_MISS       405
#define LOG_ODDS_MIN       -2000
#define LOG_ODDS_MAX        2000
#define LOG_ODDS_THRESHOLD  0

__device__ __forceinline__
int flatVoxelIndex(int ix, int iy, int iz, int NY, int NZ)
{
    return ix * NY * NZ + iy * NZ + iz;
}

// Adds delta to *addr, clamping the result to [minVal, maxVal].
__device__ void clampedAtomicAdd(int* addr, int delta, int minVal, int maxVal)
{
    int assumed, newVal, old = *addr;
    do {
        assumed = old;
        newVal  = min(max(assumed + delta, minVal), maxVal);
        if (newVal == assumed) return;           // already saturated, nothing to do
        old = atomicCAS(addr, assumed, newVal);
    } while (assumed != old);
}

extern "C"
__global__ void updateLogOddsHits(
    int*    grid,
    int     NX, int NY, int NZ,
    float   resolution,
    float   anchorX, float anchorY, float anchorZ,
    float3* points,
    int     numPoints)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;

    for (; i < numPoints; i += stride)
    {
        float3 p = points[i];

        int ix = (int)floorf((p.x - anchorX) / resolution);
        int iy = (int)floorf((p.y - anchorY) / resolution);
        int iz = (int)floorf((p.z - anchorZ) / resolution);

        if (ix < 0 || ix >= NX || iy < 0 || iy >= NY || iz < 0 || iz >= NZ)
            continue;

        clampedAtomicAdd(&grid[flatVoxelIndex(ix, iy, iz, NY, NZ)],
                         LOG_ODDS_HIT, LOG_ODDS_MIN, LOG_ODDS_MAX);
    }
}

/**
 * Casts a ray from the sensor origin through each endpoint and decrements log-odds along the ray.
 * The endpoint voxel itself is NOT updated; hits are handled by updateLogOddsHits.
 * Uses 3-D DDA traversal.
 */
extern "C"
__global__ void updateLogOddsMisses(
    int*    grid,
    int     NX, int NY, int NZ,
    float   resolution,
    float   anchorX, float anchorY, float anchorZ,
    float3* points,
    int     numPoints,
    float   sensorOriginX, float sensorOriginY, float sensorOriginZ)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;

    for (; i < numPoints; i += stride)
    {
        float3 ep = points[i];

        float dx = ep.x - sensorOriginX;
        float dy = ep.y - sensorOriginY;
        float dz = ep.z - sensorOriginZ;
        float len = sqrtf(dx*dx + dy*dy + dz*dz);
        if (len < 1e-4f) continue;

        int ix = (int)floorf((sensorOriginX - anchorX) / resolution);
        int iy = (int)floorf((sensorOriginY - anchorY) / resolution);
        int iz = (int)floorf((sensorOriginZ - anchorZ) / resolution);

        // Stop one voxel short to avoid updating the hit voxel
        int ex = (int)floorf((ep.x - anchorX) / resolution);
        int ey = (int)floorf((ep.y - anchorY) / resolution);
        int ez = (int)floorf((ep.z - anchorZ) / resolution);

        int stepX = (dx > 0) ? 1 : -1;
        int stepY = (dy > 0) ? 1 : -1;
        int stepZ = (dz > 0) ? 1 : -1;

        float invDx = (fabsf(dx) > 1e-8f) ? resolution / fabsf(dx) : 1e30f;
        float invDy = (fabsf(dy) > 1e-8f) ? resolution / fabsf(dy) : 1e30f;
        float invDz = (fabsf(dz) > 1e-8f) ? resolution / fabsf(dz) : 1e30f;

        float tMaxX = (dx > 0 ? (anchorX + (ix+1)*resolution - sensorOriginX)
                               : (sensorOriginX - (anchorX + ix*resolution))) / fabsf(dx);
        float tMaxY = (dy > 0 ? (anchorY + (iy+1)*resolution - sensorOriginY)
                               : (sensorOriginY - (anchorY + iy*resolution))) / fabsf(dy);
        float tMaxZ = (dz > 0 ? (anchorZ + (iz+1)*resolution - sensorOriginZ)
                               : (sensorOriginZ - (anchorZ + iz*resolution))) / fabsf(dz);

        while (ix != ex || iy != ey || iz != ez)
        {
            if (tMaxX < tMaxY)
            {
                if (tMaxX < tMaxZ) { ix += stepX; tMaxX += invDx; }
                else               { iz += stepZ; tMaxZ += invDz; }
            }
            else
            {
                if (tMaxY < tMaxZ) { iy += stepY; tMaxY += invDy; }
                else               { iz += stepZ; tMaxZ += invDz; }
            }

            if (ix == ex && iy == ey && iz == ez) break;
            if (ix < 0 || ix >= NX || iy < 0 || iy >= NY || iz < 0 || iz >= NZ) break;

            clampedAtomicAdd(&grid[flatVoxelIndex(ix, iy, iz, NY, NZ)],
                             -LOG_ODDS_MISS, LOG_ODDS_MIN, LOG_ODDS_MAX);
        }
    }
}

extern "C"
__global__ void clearGrid(int* grid, int totalVoxels)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (; i < totalVoxels; i += stride)
        grid[i] = 0;
}

extern "C"
__global__ void decayLogOdds(int* grid, int totalVoxels, int decayAmount)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (; i < totalVoxels; i += stride)
    {
        int v = grid[i];
        if      (v >  decayAmount) grid[i] = v - decayAmount;
        else if (v < -decayAmount) grid[i] = v + decayAmount;
        else                       grid[i] = 0;
    }
}

/**
 * Fills holes by propagating occupancy to 4-connected XY neighbours at the same Z.
 * An unknown voxel with at least {@code minOccupiedNeighbours} occupied neighbours
 * is marked weakly occupied with {@code fillLogOdds}.
 */
extern "C"
__global__ void dilateOccupancyXY(
    int* grid,
    int  NX, int NY, int NZ,
    int  minOccupiedNeighbours,
    int  fillLogOdds)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int totalVoxels = NX * NY * NZ;

    for (; i < totalVoxels; i += stride)
    {
        int iz =  i % NZ;
        int iy = (i / NZ) % NY;
        int ix =  i / (NY * NZ);

        if (grid[i] > LOG_ODDS_THRESHOLD)
            continue;

        int occupiedCount = 0;
        if (ix > 0    && grid[flatVoxelIndex(ix-1, iy, iz, NY, NZ)] > LOG_ODDS_THRESHOLD) occupiedCount++;
        if (ix < NX-1 && grid[flatVoxelIndex(ix+1, iy, iz, NY, NZ)] > LOG_ODDS_THRESHOLD) occupiedCount++;
        if (iy > 0    && grid[flatVoxelIndex(ix, iy-1, iz, NY, NZ)] > LOG_ODDS_THRESHOLD) occupiedCount++;
        if (iy < NY-1 && grid[flatVoxelIndex(ix, iy+1, iz, NY, NZ)] > LOG_ODDS_THRESHOLD) occupiedCount++;

        if (occupiedCount >= minOccupiedNeighbours)
            grid[i] = fillLogOdds;
    }
}

/**
 * Extracts a 2-D height scan for an RL policy.
 *
 * For each (scanX[i], scanY[i]) in robot base frame, transforms to world frame,
 * searches a square neighbourhood of radius {@code searchRadiusVoxels}, and reports
 * the highest occupied voxel height relative to base Z.
 * Falls back to {@code defaultRelativeHeight} when the neighbourhood is entirely unknown.
 */
extern "C"
__global__ void extractHeightScan(
    const int*   grid,
    int     NX, int NY, int NZ,
    float   resolution,
    float   anchorX, float anchorY, float anchorZ,
    const float* scanX,
    const float* scanY,
    int     numScanPoints,
    const float* baseToWorldTransform,  // row-major 4×4
    int     searchRadiusVoxels,
    float   defaultRelativeHeight,
    float*  heightsToPack)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;

    float baseZ = baseToWorldTransform[11];

    for (; i < numScanPoints; i += stride)
    {
        float bx = scanX[i];
        float by = scanY[i];

        float wx = baseToWorldTransform[0]*bx + baseToWorldTransform[1]*by + baseToWorldTransform[3];
        float wy = baseToWorldTransform[4]*bx + baseToWorldTransform[5]*by + baseToWorldTransform[7];

        int cx = (int)floorf((wx - anchorX) / resolution);
        int cy = (int)floorf((wy - anchorY) / resolution);

        float found = defaultRelativeHeight;
        int R = searchRadiusVoxels;

        for (int dr = 0; dr <= R && found == defaultRelativeHeight; dr++)
        {
            for (int dx = -dr; dx <= dr && found == defaultRelativeHeight; dx++)
            for (int dy = -dr; dy <= dr && found == defaultRelativeHeight; dy++)
            {
                if (abs(dx) != dr && abs(dy) != dr) continue; // only the shell at distance dr

                int ix = cx + dx;
                int iy = cy + dy;
                if (ix < 0 || ix >= NX || iy < 0 || iy >= NY) continue;

                for (int iz = NZ - 1; iz >= 0; iz--)
                {
                    if (grid[flatVoxelIndex(ix, iy, iz, NY, NZ)] > LOG_ODDS_THRESHOLD)
                    {
                        found = (anchorZ + (iz + 0.5f) * resolution) - baseZ;
                        break;
                    }
                }
            }
        }

        heightsToPack[i] = found;
    }
}
