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
                                     float3 rayDirection,
                                     float3 topCenter,
                                     float3 bottomCenter,
                                     float radius,
                                     float &distanceToIntersection)
{
    //distanceBetweenCenters = bottomCenter - topCenter
    float3 distanceBetweenCenters;
    distanceBetweenCenters.x = bottomCenter.x - topCenter.x;
    distanceBetweenCenters.y = bottomCenter.y - topCenter.y;
    distanceBetweenCenters.z = bottomCenter.z - topCenter.z;

    //distanceBetweenRayAndTopCenter = rayOrigin - topCenter
    float3 distanceBetweenRayAndTopCenter;
    distanceBetweenRayAndTopCenter.x = rayOrigin.x - topCenter.x;
    distanceBetweenRayAndTopCenter.y = rayOrigin.y - topCenter.y;
    distanceBetweenRayAndTopCenter.z = rayOrigin.z - topCenter.z;

    float squaredLengthOfCapsuleAxis = dot(distanceBetweenCenters, distanceBetweenCenters);

    //dot product between capsule's axis vector and the ray direction
    //if they point the same way value is large, if they are perpendicular value is 0 and if they are opposite value is negative
    float axisProjection = distanceBetweenCenters.x * rayDirection.x + distanceBetweenCenters.y * rayDirection.y + distanceBetweenCenters.z * rayDirection.z;

    //projection of the ray origin vector onto the capsule's axis
    //if its greater than 0, then the ray origin is below the top center
    //if its less than 0, then ray origin is above the top center
    float rayVectorProjection = distanceBetweenCenters.x * distanceBetweenRayAndTopCenter.x + distanceBetweenCenters.y * distanceBetweenRayAndTopCenter.y + distanceBetweenCenters.z * distanceBetweenRayAndTopCenter.z;

    //the dot product tells you how much the ray’s direction points toward or away from the capsule’s top center
    float rayDirOriginProjection = rayDirection.x * distanceBetweenRayAndTopCenter.x + rayDirection.y * distanceBetweenRayAndTopCenter.y + rayDirection.z * distanceBetweenRayAndTopCenter.z;

    float magnitudeDistanceBetweenRayAndTopCenter = dot(distanceBetweenRayAndTopCenter, distanceBetweenRayAndTopCenter);

    float quadraticCoeffA = squaredLengthOfCapsuleAxis - axisProjection * axisProjection;

    float quadraticCoeffB = squaredLengthOfCapsuleAxis * rayDirOriginProjection - rayVectorProjection * axisProjection;

    float quadraticCoeffC = squaredLengthOfCapsuleAxis * (magnitudeDistanceBetweenRayAndTopCenter - radius * radius) - rayVectorProjection * rayVectorProjection;

    float quadraticCoeffDiscriminant = quadraticCoeffB * quadraticCoeffB - quadraticCoeffA * quadraticCoeffC;


    //no real solution
    if (quadraticCoeffDiscriminant < 0.0f)
        return false;

    quadraticCoeffDiscriminant = sqrtf(quadraticCoeffDiscriminant);

    distanceToIntersection = (-quadraticCoeffB - quadraticCoeffDiscriminant) / quadraticCoeffA ;

    // Clamp hit along finite capsule
    float distanceFromTopCenterToIntersection = rayVectorProjection + distanceToIntersection * axisProjection;
    if (distanceFromTopCenterToIntersection > 0.0f && distanceFromTopCenterToIntersection < squaredLengthOfCapsuleAxis)
        return distanceToIntersection > 0.0f;

    // Check hemispherical caps
    float3 sphereCheck;
    if (distanceFromTopCenterToIntersection <= 0.0f)
    {
        sphereCheck.x = distanceBetweenRayAndTopCenter.x;
        sphereCheck.y = distanceBetweenRayAndTopCenter.y;
        sphereCheck.z = distanceBetweenRayAndTopCenter.z;
    }
    else
    {
        sphereCheck.x = rayOrigin.x - bottomCenter.x;
        sphereCheck.y = rayOrigin.y - bottomCenter.y;
        sphereCheck.z = rayOrigin.z - bottomCenter.z;
    }

    float raySphereIntersection = rayDirection.x * sphereCheck.x + rayDirection.y * sphereCheck.y + rayDirection.z * sphereCheck.z;
    float sphereQuadraticConstant = (sphereCheck.x * sphereCheck.x + sphereCheck.y * sphereCheck.y + sphereCheck.z * sphereCheck.z) - radius * radius;
    float quadraticCoeffDiscriminantSphere = raySphereIntersection * raySphereIntersection - sphereQuadraticConstant;
    if (quadraticCoeffDiscriminantSphere < 0.0f)
        return false;

    distanceToIntersection = -raySphereIntersection - sqrtf(quadraticCoeffDiscriminantSphere);
    return distanceToIntersection > 0.0f;
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

    float3 rayOrigin = make_float3(0.0f, 0.0f, 0.0f);
    float3 rayDirection = normalize(make_float3(
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
        if (rayIntersectsCapsule(rayOrigin, rayDirection, topCenter, bottomCenter, radius, tHit))
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