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
    // Dot product for bottomCenter - topCenter
    float3 distanceBetweenCenters = sub(bottomCenter, topCenter);

    // Dot product for rayOrigin - topCenter
    float3 distanceBetweenRayAndTopCenter = sub(rayOrigin, topCenter);

    float squaredLengthOfCapsuleAxis = dot(distanceBetweenCenters, distanceBetweenCenters);

    // Dot product between capsule's axis vector and the ray direction
    // If they point the same way value is large, if they are perpendicular value is 0 and if they are opposite value is negative
    float axisProjection = dot(distanceBetweenCenters, rayDirection);

    // Projection of the ray origin vector onto the capsule's axis
    // If its greater than 0, then the ray origin is below the top center
    // If its less than 0, then ray origin is above the top center
    float rayVectorProjection = dot(distanceBetweenCenters, distanceBetweenRayAndTopCenter);

    // The dot product tells you how much the ray’s direction points toward or away from the capsule’s top center
    float rayDirOriginProjection = dot(rayDirection, distanceBetweenRayAndTopCenter);

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
        sphereCheck = distanceBetweenRayAndTopCenter;
    }
    else
    {
        sphereCheck = sub(rayOrigin, bottomCenter);
    }

    float raySphereIntersection =  dot(rayDirection, sphereCheck);//rayDirection.x * sphereCheck.x + rayDirection.y * sphereCheck.y + rayDirection.z * sphereCheck.z;
    float sphereQuadraticConstant = (dot(sphereCheck, sphereCheck)) - radius * radius;
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