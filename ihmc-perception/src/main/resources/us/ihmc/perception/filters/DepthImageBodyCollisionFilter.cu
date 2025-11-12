#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

__device__ bool isPointInCapsule(float3 point,
                                 float3 topCenter,
                                 float3 bottomCenter,
                                 float radius)
{
    float tolerance = 0.1;
    float3 capsuleAxis;
    capsuleAxis.x = bottomCenter.x - topCenter.x;
    capsuleAxis.y = bottomCenter.y - topCenter.y;
    capsuleAxis.z = bottomCenter.z - topCenter.z;

    float capsuleLengthSq = dot(capsuleAxis, capsuleAxis);
    float capsuleLength = sqrtf(capsuleLengthSq);

    if (capsuleLength < 1e-6f)
    {
        float3 diff;
        diff.x = point.x - topCenter.x;
        diff.y = point.y - topCenter.y;
        diff.z = point.z - topCenter.z;
        float distSq = dot(diff, diff);
        return distSq <=  (radius + tolerance) * (radius + tolerance);

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
    return distSq <= (radius + tolerance) * (radius + tolerance);
}

__device__ bool rayIntersectsCapsule(float3 rayOrigin,
                                     float3 rayDir,
                                     float3 topCenter,
                                     float3 bottomCenter,
                                     float radius,
                                     float &tHit)
{
    // ba = bottomCenter - topCenter
    float3 ba;
    ba.x = bottomCenter.x - topCenter.x;
    ba.y = bottomCenter.y - topCenter.y;
    ba.z = bottomCenter.z - topCenter.z;

    // oa = rayOrigin - topCenter
    float3 oa;
    oa.x = rayOrigin.x - topCenter.x;
    oa.y = rayOrigin.y - topCenter.y;
    oa.z = rayOrigin.z - topCenter.z;

    float baba = dot(ba, ba);
    float bard = ba.x * rayDir.x + ba.y * rayDir.y + ba.z * rayDir.z;
    float baoa = ba.x * oa.x + ba.y * oa.y + ba.z * oa.z;
    float rdoa = rayDir.x * oa.x + rayDir.y * oa.y + rayDir.z * oa.z;
    float oaoa = dot(oa, oa);

    float a = baba - bard * bard;
    float b = baba * rdoa - baoa * bard;
    float c = baba * (oaoa - radius * radius) - baoa * baoa;
    float h = b * b - a * c;
    if (h < 0.0f)
        return false;

    h = sqrtf(h);
    tHit = (-b - h) / a;

    // Clamp hit along finite capsule
    float y = baoa + tHit * bard;
    if (y > 0.0f && y < baba)
        return tHit > 0.0f;

    // Check hemispherical caps
    float3 oc;
    if (y <= 0.0f)
    {
        oc.x = oa.x;
        oc.y = oa.y;
        oc.z = oa.z;
    }
    else
    {
        oc.x = rayOrigin.x - bottomCenter.x;
        oc.y = rayOrigin.y - bottomCenter.y;
        oc.z = rayOrigin.z - bottomCenter.z;
    }

    float b2 = rayDir.x * oc.x + rayDir.y * oc.y + rayDir.z * oc.z;
    float c2 = (oc.x * oc.x + oc.y * oc.y + oc.z * oc.z) - radius * radius;
    h = b2 * b2 - c2;
    if (h < 0.0f)
        return false;

    tHit = -b2 - sqrtf(h);
    return tHit > 0.0f;
}

extern "C" __global__ void checkBodyCollision(unsigned short* depthImage,
                                            size_t depthImagePitch,
                                            int width,
                                            int height,
                                            float fx,
                                            float fy,
                                            float cx,
                                            float cy,
                                            unsigned short* collisionMaskMap,
                                            size_t pitchCollisionMaskMap,
                                            float* collidableGeometryPointer,
                                            int numCollidables,
                                            int numberOfAttributes)
{
    int x_index = blockIdx.x * blockDim.x + threadIdx.x;
    int y_index = blockIdx.y * blockDim.y + threadIdx.y;
    if (x_index >= width || y_index >= height) return;

    unsigned short depthValue = *row(col(depthImage, x_index), depthImagePitch, y_index);
    float depthInMeters = depthValue / 1000.0f;

    float3 rayOrigin = make_float3(0.0f, -0.045f, 0.0f);
    float3 rayDir = normalize(make_float3(
        depthInMeters,
        -(x_index - cx) / fx * depthInMeters,
        -(y_index - cy) / fy * depthInMeters));

    unsigned short *maskPtr = (unsigned short *)((char *)collisionMaskMap + y_index * pitchCollisionMaskMap) + x_index;
    *maskPtr = depthValue;

    for (int i = 0; i < numCollidables; ++i)
    {
        int index = i * numberOfAttributes;

        float3 topCenter = make_float3(collidableGeometryPointer[index],
                                       collidableGeometryPointer[index + 1],
                                       collidableGeometryPointer[index + 2]);
        float3 bottomCenter = make_float3(collidableGeometryPointer[index + 3],
                                          collidableGeometryPointer[index + 4],
                                          collidableGeometryPointer[index + 5]);
        float radius = collidableGeometryPointer[index + 6];

        // Direct collision
        float3 point = make_float3(depthInMeters,
                                   -(x_index - cx) / fx * depthInMeters,
                                   -(y_index - cy) / fy * depthInMeters);
        if (isPointInCapsule(point, topCenter, bottomCenter, radius))
        {
            *maskPtr = 0;
            return;
        }

        // Ray intersection test
        float tHit;
        if (rayIntersectsCapsule(rayOrigin, rayDir, topCenter, bottomCenter, radius, tHit))
        {
            float hitDepth = tHit; // meters
            if (hitDepth <= depthInMeters)
            {
                *maskPtr = 0;
                return;
            }
        }
    }
}