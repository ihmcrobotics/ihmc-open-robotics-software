#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

/**
 * Extracts valid 3D world-frame points and their sampled RGB colours from two ZED X Mini
 * depth cameras simultaneously.
 *
 * Depth and left-colour are aligned by the ZED SDK, so pixel (x, y) in the depth image
 * corresponds directly to pixel (x, y) in the left-colour image.
 *
 * Thread dispatch:
 *   blockIdx.z == 0  →  camera 0: stepping camera (belly-mounted, 4 mm lens)
 *   blockIdx.z == 1  →  camera 1: experimental camera (head-mounted, 2.2 mm lens)
 *
 * Each camera writes to its own point cloud buffer and colour buffer via atomic counters.
 * Points outside the per-camera [minRange, maxRange] interval and zero-depth pixels
 * are discarded.
 */
extern "C"
__global__ void extractFusedPointCloud(
    // ── Camera 0: stepping camera (belly, 4 mm) ──────────────────────────────
    unsigned short* depthImage0, size_t depthPitch0,
    int width0, int height0,
    float fx0, float fy0, float cx0, float cy0,
    float depthDiscretization0,
    float* transform0,
    float minRange0, float maxRange0,
    unsigned char* colorImage0, size_t colorPitch0, int colorChannels0,
    float3* pointCloud0, float3* colorCloud0, int* pointCloudSize0,

    // ── Camera 1: experimental camera (head, 2.2 mm) ─────────────────────────
    unsigned short* depthImage1, size_t depthPitch1,
    int width1, int height1,
    float fx1, float fy1, float cx1, float cy1,
    float depthDiscretization1,
    float* transform1,
    float minRange1, float maxRange1,
    unsigned char* colorImage1, size_t colorPitch1, int colorChannels1,
    float3* pointCloud1, float3* colorCloud1, int* pointCloudSize1
)
{
    int startX  = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();
    int startY  = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    if (blockIdx.z == 0)
    {
        // ── Stepping camera ───────────────────────────────────────────────────
        for (int y = startY; y < height0; y += strideY)
        {
            for (int x = startX; x < width0; x += strideX)
            {
                unsigned short rawDepth = *row(col(depthImage0, x), depthPitch0, y);
                if (rawDepth == 0)
                    continue;

                float depthM = depthDiscretization0 * rawDepth;
                if (depthM < minRange0 || depthM > maxRange0)
                    continue;

                float3 worldPt = transformPoint3D(
                    pixelDepthToPoint3D(x, y, depthM, fx0, fy0, cx0, cy0), transform0);

                // Sample BGR colour at the same aligned pixel
                unsigned char* colorRow = colorImage0 + (size_t)y * colorPitch0;
                float b = colorRow[(size_t)x * colorChannels0 + 0] / 255.0f;
                float g = colorRow[(size_t)x * colorChannels0 + 1] / 255.0f;
                float r = colorRow[(size_t)x * colorChannels0 + 2] / 255.0f;

                int idx = atomicAdd(pointCloudSize0, 1);
                pointCloud0[idx] = worldPt;
                colorCloud0[idx] = make_float3(r, g, b);
            }
        }
    }
    else
    {
        // ── Experimental camera ───────────────────────────────────────────────
        for (int y = startY; y < height1; y += strideY)
        {
            for (int x = startX; x < width1; x += strideX)
            {
                unsigned short rawDepth = *row(col(depthImage1, x), depthPitch1, y);
                if (rawDepth == 0)
                    continue;

                float depthM = depthDiscretization1 * rawDepth;
                if (depthM < minRange1 || depthM > maxRange1)
                    continue;

                float3 worldPt = transformPoint3D(
                    pixelDepthToPoint3D(x, y, depthM, fx1, fy1, cx1, cy1), transform1);

                unsigned char* colorRow = colorImage1 + (size_t)y * colorPitch1;
                float b = colorRow[(size_t)x * colorChannels1 + 0] / 255.0f;
                float g = colorRow[(size_t)x * colorChannels1 + 1] / 255.0f;
                float r = colorRow[(size_t)x * colorChannels1 + 2] / 255.0f;

                int idx = atomicAdd(pointCloudSize1, 1);
                pointCloud1[idx] = worldPt;
                colorCloud1[idx] = make_float3(r, g, b);
            }
        }
    }
}
