#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

/*
 * Fuses one depth image into a voxel occupancy map.
 *
 * The map is a flattened mapSizeX * mapSizeY * mapSizeZ float array of occupancy values in {0.0, 1.0},
 * flattened x-major, z-fastest: flatIndex = (voxelX * mapSizeY + voxelY) * mapSizeZ + voxelZ.
 * Voxel indices address voxel centers: index i covers map-frame coordinates ((i - (size - 1) / 2) +/- 0.5) * voxelSize
 * along its axis, so the map is centered on the map frame's origin.
 */
extern "C"
__global__ void fuseDepthImageIntoVoxelMap(unsigned short* depthImage,
                                           size_t pitch,
                                           int width,
                                           int height,
                                           float fx,
                                           float fy,
                                           float cx,
                                           float cy,
                                           float depthDiscretization,
                                           float* depthToMapTransform,
                                           int mapSizeX,
                                           int mapSizeY,
                                           int mapSizeZ,
                                           float voxelSize,
                                           float* voxelMap)
{
    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short depthValue = *row(col(depthImage, x), pitch, y);
            if (depthValue == 0) // 0 depth = no measurement for this pixel
                continue;

            float depthInMeters = depthDiscretization * depthValue;
            float3 depthFramePoint = pixelDepthToPoint3D(x, y, depthInMeters, fx, fy, cx, cy);
            float3 mapFramePoint = transformPoint3D(depthFramePoint, depthToMapTransform);

            // Round to nearest since voxel indices address voxel centers
            int voxelX = __float2int_rn(mapFramePoint.x / voxelSize + 0.5f * (mapSizeX - 1));
            int voxelY = __float2int_rn(mapFramePoint.y / voxelSize + 0.5f * (mapSizeY - 1));
            int voxelZ = __float2int_rn(mapFramePoint.z / voxelSize + 0.5f * (mapSizeZ - 1));

            if (voxelX < 0 || voxelX >= mapSizeX || voxelY < 0 || voxelY >= mapSizeY || voxelZ < 0 || voxelZ >= mapSizeZ)
                continue; // point lands outside the local map

            // Racing threads only ever write the same value, so no atomics needed
            voxelMap[(voxelX * mapSizeY + voxelY) * mapSizeZ + voxelZ] = 1.0f;
        }
    }
}
