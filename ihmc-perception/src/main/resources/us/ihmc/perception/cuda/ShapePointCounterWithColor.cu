#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

extern "C"
__global__ void countPointsInSphere(const unsigned short* depthImage,
                                    size_t depthPitch,
                                    const unsigned char* colorImage,
                                    size_t colorPitch,
                                    int width,
                                    int height,
                                    int colorChannels,
                                    int redOffset,
                                    int greenOffset,
                                    int blueOffset,
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
                                    unsigned long long* count,
                                    unsigned long long* redSum,
                                    unsigned long long* greenSum,
                                    unsigned long long* blueSum)
{
    extern __shared__ unsigned long long threadTotals[];
    unsigned long long threadCount = 0;
    unsigned long long threadRedSum = 0;
    unsigned long long threadGreenSum = 0;
    unsigned long long threadBlueSum = 0;

    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    float r2 = sphereR * sphereR;

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short depthValue = *row(col(depthImage, x), depthPitch, y);
            if (depthValue == 0)
                continue;

            float depthInMeters = depthDiscretization * depthValue;
            float3 depthFramePoint = pixelDepthToPoint3D(x, y, depthInMeters, fx, fy, cx, cy);
            float3 worldFramePoint = transformPoint3D(depthFramePoint, depthToWorldTransform);

            float dx = worldFramePoint.x - sphereX;
            float dy = worldFramePoint.y - sphereY;
            float dz = worldFramePoint.z - sphereZ;

            if (dx * dx + dy * dy + dz * dz <= r2)
            {
                const unsigned char* colorRow = row(colorImage, colorPitch, y);
                int colorIndex = x * colorChannels;

                ++threadCount;
                threadRedSum += colorRow[colorIndex + redOffset];
                threadGreenSum += colorRow[colorIndex + greenOffset];
                threadBlueSum += colorRow[colorIndex + blueOffset];
            }
        }
    }

    Utils::reduceAdd(threadCount, threadTotals, count);
    Utils::reduceAdd(threadRedSum, threadTotals, redSum);
    Utils::reduceAdd(threadGreenSum, threadTotals, greenSum);
    Utils::reduceAdd(threadBlueSum, threadTotals, blueSum);
}

extern "C"
__global__ void countPointsInCapsule(const unsigned short* depthImage,
                                     size_t depthPitch,
                                     const unsigned char* colorImage,
                                     size_t colorPitch,
                                     int width,
                                     int height,
                                     int colorChannels,
                                     int redOffset,
                                     int greenOffset,
                                     int blueOffset,
                                     float fx,
                                     float fy,
                                     float cx,
                                     float cy,
                                     float depthDiscretization,
                                     const float* depthToWorldTransform,
                                     const float* capsulePoints,
                                     float capsuleRadius,
                                     unsigned long long* count,
                                     unsigned long long* redSum,
                                     unsigned long long* greenSum,
                                     unsigned long long* blueSum)
{
    extern __shared__ unsigned long long threadTotals[];
    unsigned long long threadCount = 0;
    unsigned long long threadRedSum = 0;
    unsigned long long threadGreenSum = 0;
    unsigned long long threadBlueSum = 0;

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
            unsigned short depthValue = *row(col(depthImage, x), depthPitch, y);
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
            {
                const unsigned char* colorRow = row(colorImage, colorPitch, y);
                int colorIndex = x * colorChannels;

                ++threadCount;
                threadRedSum += colorRow[colorIndex + redOffset];
                threadGreenSum += colorRow[colorIndex + greenOffset];
                threadBlueSum += colorRow[colorIndex + blueOffset];
            }
        }
    }

    Utils::reduceAdd(threadCount, threadTotals, count);
    Utils::reduceAdd(threadRedSum, threadTotals, redSum);
    Utils::reduceAdd(threadGreenSum, threadTotals, greenSum);
    Utils::reduceAdd(threadBlueSum, threadTotals, blueSum);
}
