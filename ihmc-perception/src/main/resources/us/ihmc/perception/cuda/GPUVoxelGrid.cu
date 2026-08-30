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
    float*  pointWeights,   // per-point confidence in (0, 1]; may be null for full-weight hits
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

        // Scale the hit log-odds by the measurement confidence: distant (higher-variance)
        // points nudge occupancy less, so the nearer-ranging camera dominates the fused map.
        int hitLogOdds = (pointWeights == nullptr)
                       ? LOG_ODDS_HIT
                       : (int)lroundf(LOG_ODDS_HIT * pointWeights[i]);
        if (hitLogOdds <= 0)
            continue;

        clampedAtomicAdd(&grid[flatVoxelIndex(ix, iy, iz, NY, NZ)],
                         hitLogOdds, LOG_ODDS_MIN, LOG_ODDS_MAX);
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
    float*  pointWeights,   // per-point confidence in (0, 1]; may be null for full-strength misses
    int     numPoints,
    float   sensorOriginX, float sensorOriginY, float sensorOriginZ)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;

    for (; i < numPoints; i += stride)
    {
        // Scale the miss log-odds by the same per-point confidence used for the hit at this
        // point's endpoint, so a weak/distant detection doesn't get out-voted by full-strength
        // misses along its own ray (hits were already discounted by range; misses weren't, which
        // let unweighted erosion dominate weak-but-legitimate distant hits).
        int missLogOdds = (pointWeights == nullptr)
                         ? LOG_ODDS_MISS
                         : (int)lroundf(LOG_ODDS_MISS * pointWeights[i]);
        if (missLogOdds <= 0)
            continue;

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
                             -missLogOdds, LOG_ODDS_MIN, LOG_ODDS_MAX);
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
 * Extracts a robot-centric, yaw-aligned 3-D binary occupancy crop from the persistent world grid,
 * for use as an RL observation (Gallant-style: arXiv:2511.14625).
 *
 * Unlike the height scan, this preserves full vertical / multi-layer structure (walls, overhangs,
 * undersides), so the policy can reason about pushing off vertical surfaces during fall recovery.
 *
 * The persistent grid is world-anchored (it retains geometry mapped before a fall); this kernel
 * samples a crop that is translated to the robot position and rotated by the robot yaw only
 * (gravity-aligned z), so the observation stays upright even when the robot is prone.
 *
 * Crop local axes: x forward, y left, z up, centred on the robot. Output layout is z-as-channel
 * (matching the paper's z-grouped 2D CNN): out[cz*(cropNY*cropNX) + cy*cropNX + cx].
 * A voxel reads 1.0 when occupied (log-odds > threshold), else 0.0 (unknown and free both read 0).
 */
extern "C"
__global__ void extractVoxelOccupancyCrop(
    const int* grid,
    int     NX, int NY, int NZ,
    float   resolution,
    float   anchorX, float anchorY, float anchorZ,
    float   robotX, float robotY, float robotZ,   // crop centre in world frame
    float   cosYaw, float sinYaw,                  // robot yaw (gravity-aligned crop)
    int     cropNX, int cropNY, int cropNZ,
    float   cropResolution,
    float*  occupancyToPack)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;

    int cropTotal = cropNX * cropNY * cropNZ;

    float halfX = 0.5f * (cropNX - 1);
    float halfY = 0.5f * (cropNY - 1);
    float halfZ = 0.5f * (cropNZ - 1);

    for (; i < cropTotal; i += stride)
    {
        int cx =  i % cropNX;
        int cy = (i / cropNX) % cropNY;
        int cz =  i / (cropNX * cropNY);

        // Crop-local position (robot frame), centred on the robot.
        float lx = (cx - halfX) * cropResolution;
        float ly = (cy - halfY) * cropResolution;
        float lz = (cz - halfZ) * cropResolution;

        // Rotate by yaw (z stays gravity-aligned) and translate to world.
        float wx = robotX + cosYaw * lx - sinYaw * ly;
        float wy = robotY + sinYaw * lx + cosYaw * ly;
        float wz = robotZ + lz;

        int ix = (int)floorf((wx - anchorX) / resolution);
        int iy = (int)floorf((wy - anchorY) / resolution);
        int iz = (int)floorf((wz - anchorZ) / resolution);

        // Emit occupancy PROBABILITY, not a hard 0/1, so the policy can tell apart
        //   ~0.5 = unknown (no evidence, or outside the world grid),
        //   ->0  = confidently free (carved by ray misses),
        //   ->1  = confidently occupied (confidence-weighted hits from both cameras).
        // Log-odds are fixed-point (scale 1000); probability = sigmoid(logOdds).
        float probability = 0.5f; // unknown / out-of-bounds
        if (ix >= 0 && ix < NX && iy >= 0 && iy < NY && iz >= 0 && iz < NZ)
        {
            float logOdds = grid[flatVoxelIndex(ix, iy, iz, NY, NZ)] / 1000.0f;
            probability = 1.0f / (1.0f + expf(-logOdds));
        }

        occupancyToPack[cz * (cropNY * cropNX) + cy * cropNX + cx] = probability;
    }
}

/**
 * Collects all occupied-leaning voxels (log-odds > threshold) of the whole world grid into compact
 * world-frame position + occupancy-probability buffers, for visualising the entire fused map in RDX.
 * Probability = sigmoid(log-odds) so the viewer can colour by confidence rather than a flat binary.
 */
extern "C"
__global__ void extractOccupiedVoxelsWithProbability(
    const int* grid,
    int     NX, int NY, int NZ,
    float   resolution,
    float   anchorX, float anchorY, float anchorZ,
    float3* outPositions,
    float*  outProbabilities,
    int*    outCount,
    int     maxVoxels)
{
    int i      = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    int total  = NX * NY * NZ;

    for (; i < total; i += stride)
    {
        int logOddsFixed = grid[i];
        if (logOddsFixed <= LOG_ODDS_THRESHOLD)
            continue; // only occupied-leaning voxels are drawn

        int iz =  i % NZ;
        int iy = (i / NZ) % NY;
        int ix =  i / (NY * NZ);

        int slot = atomicAdd(outCount, 1);
        if (slot >= maxVoxels)
            continue;

        outPositions[slot] = make_float3(anchorX + (ix + 0.5f) * resolution,
                                         anchorY + (iy + 0.5f) * resolution,
                                         anchorZ + (iz + 0.5f) * resolution);
        outProbabilities[slot] = 1.0f / (1.0f + expf(-(logOddsFixed / 1000.0f)));
    }
}
