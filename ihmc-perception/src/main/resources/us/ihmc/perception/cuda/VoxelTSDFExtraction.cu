#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

/*
 * Fuses one 16-bit depth image into a robot-centric TSDF (truncated signed-distance field) voxel map.
 *
 * Grid layout: N^3 floats, x-major z-fastest -- flatIndex = (voxelX * N + voxelY) * N + voxelZ.
 * Voxel indices are half-voxel-centred: voxel i covers map-frame coordinate (i - (N-1)/2) * voxelSize.
 *
 * TSDF value convention (pelvis-anchored, matching depth_voxel_tsdf.py):
 *   tsdf = clamp((dist(surface, pelvis) - dist(voxel_center, pelvis)) / truncationDistance, -1, 1)
 *
 *   +1 = free / unobserved (voxel is nearer to pelvis than observed surface, or not in camera frustum)
 *   -1 = occluded (voxel is farther from pelvis than the observed surface)
 *
 * Since the pelvis is at the map-frame origin, both distances are simple Euclidean norms in map frame:
 *   tsdf = clamp((||surface_point_map|| - ||voxel_center_map||) / truncationDistance, -1, 1)
 *
 * Multi-camera fusion: call this kernel once per depth image (same stream, same buffers).  The kernel
 * does a min-update on voxelMap (most-occluded value wins across cameras), matching Python's
 *   fused_tsdf = where(valid_cam, minimum(fused_tsdf, tsdf_cam), fused_tsdf)
 * and sets validMap[i] = 1 wherever this camera provides a measurement, implementing the OR fusion
 * for any_valid across cameras.  Voxels not seen by this camera are left unchanged in both buffers.
 *
 * Dispatch: 1D, one thread per voxel.  gridSize = ceil(N^3 / blockSize).
 *
 * Transforms (4x4 row-major float arrays, packed as {R00,R01,R02,Tx, R10,R11,R12,Ty, R20,R21,R22,Tz}):
 *   mapToDepthTransform: map frame → IHMC depth sensor frame (X forward, Y left, Z up)
 *   depthToMapTransform: IHMC depth sensor frame → map frame  (precomputed inverse of the above)
 *
 * depthValue == 0 means "no measurement" (standard 16-bit depth convention).
 */

static const float FREE_SPACE_VALUE = 1.0f;
static const float MIN_CAMERA_DEPTH_M = 0.05f;

extern "C"
__global__ void fuseDepthImageToTSDF(unsigned short* depthImage,
                                     size_t pitch,
                                     int width,
                                     int height,
                                     float fx,
                                     float fy,
                                     float cx,
                                     float cy,
                                     float depthDiscretization,
                                     float* mapToDepthTransform,
                                     float* depthToMapTransform,
                                     int N,
                                     float voxelSize,
                                     float truncationDistance,
                                     float* voxelMap,
                                     char* validMap)
{
    int flatIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (flatIdx >= N * N * N)
        return;

    // Decompose flat index: x-major, z-fastest
    int voxelX = flatIdx / (N * N);
    int voxelY = (flatIdx / N) % N;
    int voxelZ = flatIdx % N;

    // Voxel center in map frame (pelvis at origin, half-voxel-centred indexing)
    float3 voxel_map = make_float3((voxelX - 0.5f * (N - 1)) * voxelSize,
                                   (voxelY - 0.5f * (N - 1)) * voxelSize,
                                   (voxelZ - 0.5f * (N - 1)) * voxelSize);
    float voxel_dist = length(voxel_map); // ||voxel_center_map|| since pelvis is at origin

    // Transform voxel center to depth sensor frame (IHMC: X forward, Y left, Z up)
    float3 voxel_sensor = transformPoint3D(voxel_map, mapToDepthTransform);

    // Voxel must be in front of camera (positive depth along X axis in IHMC sensor frame)
    if (voxel_sensor.x <= MIN_CAMERA_DEPTH_M)
        return;

    // Project to pixel.  IHMC sensor frame is X-forward / Y-left / Z-up, so:
    //   pixel column: px = -(Y/X) * fx + cx   (Y-left means increasing-column when negated)
    //   pixel row:    py = -(Z/X) * fy + cy   (Z-up means increasing-row when negated)
    // This is the algebraic inverse of pixelDepthToPoint3D(px,py,depth) = (depth, -(px-cx)/fx*depth, -(py-cy)/fy*depth).
    float u = -voxel_sensor.y / voxel_sensor.x * fx + cx;
    float v = -voxel_sensor.z / voxel_sensor.x * fy + cy;

    int px = __float2int_rn(u);
    int py = __float2int_rn(v);

    if (px < 0 || px >= width || py < 0 || py >= height)
        return;

    unsigned short depthValue = *row(col(depthImage, px), pitch, py);
    if (depthValue == 0) // 0 = no measurement
        return;

    float depth = depthDiscretization * depthValue;

    // Backproject pixel to sensor frame, then to map frame.
    // pixelDepthToPoint3D(px, py, depth) = (depth, -(px-cx)/fx*depth, -(py-cy)/fy*depth)
    float3 surface_sensor = pixelDepthToPoint3D(px, py, depth, fx, fy, cx, cy);
    float3 surface_map = transformPoint3D(surface_sensor, depthToMapTransform);

    float surface_dist = length(surface_map); // ||surface_point_map|| since pelvis is at origin

    // Pelvis-anchored signed distance (matches depth_voxel_tsdf.py's convention exactly)
    float signed_dist = surface_dist - voxel_dist;
    float tsdf = clamp(signed_dist / truncationDistance, -1.0f, 1.0f);

    // Min fusion: most-occluded reading wins (matches Python: fused_tsdf = min(fused_tsdf, tsdf_cam))
    voxelMap[flatIdx] = fminf(voxelMap[flatIdx], tsdf);
    validMap[flatIdx] = 1; // mark voxel as observed by at least one camera
}
