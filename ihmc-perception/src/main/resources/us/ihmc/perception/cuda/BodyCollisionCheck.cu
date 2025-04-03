#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

__device__ bool isPointInCapsule(float3 point,
                                 float3 topCenter,
                                 float3 bottomCenter,
                                 float radius) {
    float3 capsuleAxis;
    capsuleAxis.x = bottomCenter.x - topCenter.x;
    capsuleAxis.y = bottomCenter.y - topCenter.y;
    capsuleAxis.z = bottomCenter.z - topCenter.z;

    float capsuleLengthSq = dot(capsuleAxis, capsuleAxis);
    float capsuleLength = sqrtf(capsuleLengthSq);

    if (capsuleLength < 1e-6f) {
        float3 diff;
        diff.x = point.x - topCenter.x;
        diff.y = point.y - topCenter.y;
        diff.z = point.z - topCenter.z;
        float distSq = dot(diff, diff);
        return distSq <= radius * radius;
    }

    float3 normalizedAxis;
    normalizedAxis.x = capsuleAxis.x / capsuleLength;
    normalizedAxis.y = capsuleAxis.y / capsuleLength;
    normalizedAxis.z = capsuleAxis.z / capsuleLength;

    float3 toPoint;
    toPoint.x = point.x - topCenter.x;
    toPoint.y = point.y - topCenter.y;
    toPoint.z = point.z - topCenter.z;

    float projection = dot(toPoint, normalizedAxis);
    projection = fmaxf(0.0f, fminf(projection, capsuleLength));

    float3 closestPointOnAxis;
    closestPointOnAxis.x = topCenter.x + projection * normalizedAxis.x;
    closestPointOnAxis.y = topCenter.y + projection * normalizedAxis.y;
    closestPointOnAxis.z = topCenter.z + projection * normalizedAxis.z;

    float3 dist;
    dist.x = point.x - closestPointOnAxis.x;
    dist.y = point.y - closestPointOnAxis.y;
    dist.z = point.z - closestPointOnAxis.z;

    float distSq = dot(dist, dist);
    return distSq <= radius * radius;

}

extern "C" __global__ void checkBodyCollision(unsigned short* depthImage,
                                            size_t depthImagePitch,
                                            int width,
                                            int height,
                                            float fx,
                                            float fy,
                                            float cx,
                                            float cy,
                                            unsigned short* collisionMask,
                                            size_t collisionMaskPitch,
                                            float* collidableGeometryPointer,
                                            int numCollidables) {
    int x = Utils::getThreadCoordX();
    int y = Utils::getThreadCoordY();

    if (x >= width || y >= height)
        return;

    unsigned short depthValue = *row(col(depthImage, x), depthImagePitch, y);
    if (depthValue == 0) {
        *row(col(collisionMask, x), collisionMaskPitch, y) = 0; // Set to 0 (integer)
        return;
    }

    float depthInMeters = depthValue/1000.0f;
    float3 depthFramePoint = make_float3(depthInMeters,
                                          -(x - cx) / fx * depthInMeters,
                                         -(y - cy) / fy * depthInMeters);

    *row(col(collisionMask, x), collisionMaskPitch, y) = 0;

    for (int i = 0; i < numCollidables; ++i)
    {
        int index = i * 7;

        float3 topCenter = make_float3(collidableGeometryPointer[index],
                                       collidableGeometryPointer[index + 1],
                                       collidableGeometryPointer[index + 2]);



        float3 bottomCenter = make_float3(collidableGeometryPointer[index + 3],
                                          collidableGeometryPointer[index + 4],
                                          collidableGeometryPointer[index + 5]);

        float radius = collidableGeometryPointer[index + 6];

        if (isPointInCapsule(depthFramePoint, topCenter, bottomCenter, radius))
        {
            *row(col(collisionMask, x), collisionMaskPitch, y) = 255; // Set to 255 (integer)
            return; // No need to check other capsules if a collision is found
        }
    }
}