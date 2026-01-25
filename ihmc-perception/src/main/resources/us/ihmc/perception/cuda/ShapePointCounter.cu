#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

extern "C"
__global__ void countPointsInSphere(unsigned short* depthImage,
                                    size_t pitch,
                                    int width,
                                    int height,
                                    float fx,
                                    float fy,
                                    float cx,
                                    float cy,
                                    float depthDiscretization,
                                    float* depthToWorldTransform,
                                    float sphereX,
                                    float sphereY,
                                    float sphereZ,
                                    float sphereR,
                                    int* count)
{
    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    if (startY == 0 && startX == 0)
        *count = 0;
    __syncthreads();

    float r2 = sphereR * sphereR;

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short depthValue = *row(col(depthImage, x), pitch, y);
            if (depthValue == 0)
                continue;

            float depthInMeters = depthDiscretization * depthValue;
            float3 depthFramePoint = make_float3(depthInMeters,
                                                 -(x - cx) / fx * depthInMeters,
                                                 -(y - cy) / fy * depthInMeters);
            float3 worldFramePoint = transformPoint3D(depthFramePoint, depthToWorldTransform);

            float dx = worldFramePoint.x - sphereX;
            float dy = worldFramePoint.y - sphereY;
            float dz = worldFramePoint.z - sphereZ;

            if (dx * dx + dy * dy + dz * dz <= r2)
                atomicAdd(count, 1);
        }
    }
}
