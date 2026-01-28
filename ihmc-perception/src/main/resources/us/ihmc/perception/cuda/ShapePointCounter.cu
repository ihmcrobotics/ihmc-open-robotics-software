#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

template <typename T>
__device__ void reduceSum(T thisThreadValue, T* __restrict__ sharedArray, T* __restrict__ globalResult)
{
    unsigned int blockSize = blockDim.x * blockDim.y * blockDim.z;
    unsigned int threadBlockIndex = threadIdx.x
                                  + threadIdx.y * blockDim.x
                                  + threadIdx.z * blockDim.x * blockDim.y;

    sharedArray[threadBlockIndex] = thisThreadValue;
    __syncthreads();

    for (unsigned int stride = blockSize / 2; stride > 0; stride >>= 1)
    {
        if (threadBlockIndex < stride)
            sharedArray[threadBlockIndex] += sharedArray[threadBlockIndex + stride];

        __syncthreads();
    }

    if (threadBlockIndex == 0)
        atomicAdd(globalResult, sharedArray[0]);
}

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
                                    unsigned int* count)
{
    extern __shared__ unsigned int threadCounts[];
    unsigned int threadCount = 0;

    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    float r2 = sphereR * sphereR;

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short depthValue = *row(col(depthImage, x), pitch, y);
            if (depthValue == 0)
                continue;

            float depthInMeters = depthDiscretization * depthValue;
            float3 depthFramePoint = pixelDepthToPoint3D(x, y, depthInMeters, fx, fy, cx, cy);
            float3 worldFramePoint = transformPoint3D(depthFramePoint, depthToWorldTransform);

            float dx = worldFramePoint.x - sphereX;
            float dy = worldFramePoint.y - sphereY;
            float dz = worldFramePoint.z - sphereZ;

            if (dx * dx + dy * dy + dz * dz <= r2)
                ++threadCount;
        }
    }

    reduceSum(threadCount, threadCounts, count);
}

extern "C"
__global__ void countPointsInCapsule(const unsigned short* depthImage,
                                     size_t pitch,
                                     int width,
                                     int height,
                                     float fx,
                                     float fy,
                                     float cx,
                                     float cy,
                                     float depthDiscretization,
                                     const float* depthToWorldTransform,
                                     const float* capsulePoints,
                                     float capsuleRadius,
                                     unsigned int* count)
{
    extern __shared__ unsigned int threadCounts[];
    unsigned int threadCount = 0;

    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    float3 capsulePointA = make_float3(capsulePoints[0], capsulePoints[1], capsulePoints[2]);
    float3 capsulePointB = make_float3(capsulePoints[3], capsulePoints[4], capsulePoints[5]);

    // Vector from A to B
    float3 ab = capsulePointB - capsulePointA;
    float abLength2 = normSquared(ab);
    float radius2 = capsuleRadius * capsuleRadius;

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short depthValue = *row(col(depthImage, x), pitch, y);
            if (depthValue == 0)
                continue;

            float depthInMeters = depthDiscretization * depthValue;
            float3 depthFramePoint = pixelDepthToPoint3D(x, y, depthInMeters, fx, fy, cx, cy);
            float3 worldFramePoint = transformPoint3D(depthFramePoint, depthToWorldTransform);

            float distance2;

            // Vector from A to point P
            float3 ap = worldFramePoint - capsulePointA;

            if (abLength2 < EPSILON_F)
            {
                // Case if capsule is pretty much a sphere (distance between A and B ~= 0)
                distance2 = normSquared(ap);
            }
            else
            {
                // Project AP onto AB to get parameter t on the segment
                float t = clamp(dot(ap, ab) / abLength2, 0.0f, 1.0f);

                // Closest point C on segment AB to P
                float3 c = capsulePointA + (ab * t);

                // Squared distance from P to C
                float3 pc = worldFramePoint - c;
                distance2 = normSquared(pc);
            }

            if (distance2 <= radius2)
                ++threadCount;
        }
    }

    reduceSum(threadCount, threadCounts, count);
}
