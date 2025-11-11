#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;


__device__ float getDepthOfClosestCollisionOnSphere(float3 direction, float3 sphereOrigin, float radius)
{
    // This follows the description here:
    // https://math.stackexchange.com/questions/1939423/calculate-if-vector-intersects-sphere
    // The difference is that the point P is zero. If we then normalize the direction, the parameter t is equivalent to the distance along the line.
    float3 U = normalize(direction);
    float3 Q = scale(-1.0, sphereOrigin);

    float a = dot(U, U);
    float b = 2.0 * dot(U, Q);
    float c = dot(Q, Q) - radius * radius;

    float discriminant = b * b - 4.0 * a * c;
    if (d < 0.0)
    { // there are no intersections with the sphere, return
        return INFINITY;
    }
    if (d == 0.0)
    { // there's only one intersection with the sphere, since the two solutions are identical
        return -b / (2.0 * a);
    }

    // Two intersections, return the closest one.
    float delta = sqrtf(discriminant);
    float solution1 = (-b + delta) / (2.0 * a);
    float solution2 = (-b - delta) / (2.0 * a);

    return fminf(solution1, solution2);
}

__device__ float percentageAlongLine3D(float3 queryPoint, float3 pointOnLine, float3 lineDirection)
{
    float3 delta = sub(queryPoint, pointOnLine);
    float dot = dot(delta, lineDirection);
    return dot / dot(lineDirection, lineDirection);
}

__device__ float getDepthOfClosestCollisionOnCylinder(float3 direction, float3 topCenter, float3 bottomCenter, float radius)
{
    float3 cylinderAxis = sub(bottomCenter, topCenter);
    float cylinderLengthSq = dot(cylinderAxis, cylinderAxis);
    float cylinderLength = sqrtf(cylinderLengthSq);
    float radiusSquared = radius * radius;

    if (cylinderLength < 1e-6f)
    {
        return INFINITY;
    }

    float3 normalizedCylinderAxis = scale(1.0 / cylinderLength, cylinderAxis);
    float3 normalizedDirection = normalize(direction);

    float lineDirectionDotCylinderAxis = dot(normalizedDirection, normalizedCylinderAxis);

    float dIntersection1 = nanf("");
    float dIntersection2 = nanf("");

    if (fabsf(lineDirectionDotCylinderAxis) > 1e-9f)
    {
        float dTop = nanf("");
        {   // Compute the intersection with the top face using line-plane intersection
            float numerator = dot(topCenter, normalizedCylinderAxis);
            dTop = numerator / lineDirectionDotCylinderAxis;
            float3 intersection = scale(dTop, direction);
            if (distanceSquared(intersection, topCenter) > radiusSquared)
                dTop = nanf("");
        }

        if (!isnanf(dTop))
            dIntersection1 = dTop;

        float dBottom = nanf("");
        { // Compute hte intersection with the bottom face using simplified line-plane intersection
            float numerator = dot(bottom, normalizedCylinderAxis);
            dBottom = numerator / lineDirectionDotCylinderAxis;
            float3 intersection = scale(dBottom, direction);
            if (distanceSquared(intersection, bottomCenter) > radiusSquared)
                dBottom = nanf("");
        }

        if (!isnanf(dBottom))
        {
            if (isnanf(dIntersection1))
            {
                dIntersection1 = dBottom;
            }
            else if (dBottom < dIntersection1)
            {
                dIntersection2 = dIntersection1;
                dIntersection1 = dBottom;
            }
            else
            {
                dIntersection2 = dBottom;
            }
        }
    }

    // If dIntersection2 is non NaN, that means two intersections were found, which is the max, so no need to check the cylinder part.
    if (isnanf(dIntersection2))
    { // Compute the possible intersections with the cylinder part
        // Notation used: cylinder axis: pa + va * d; line equation = v * d
        // Need to solve quadratic equation of the form A * d^2 + B * d + C = 0;

        float3 cylinderPosition = scale(0.5, add(topCenter, bottomCenter));
        // (v, va) * va
        float3 scaledAxis = scale(lineDirectionDotCylinderAxis, normalizedCylinderAxis);
        // Vector used for computing A and B: v - (v, va) * va
        float3 A_vector = sub(direction, scaledAxis);
        // Delta_p
        float3 deltaP = scale(-1.0f, cylinderPosition);
        // (Delta_p, va)
        float3 scaledAxis = scale(dot(deltaP, normalizedCylinderAxis), normalizedCylinderAxis);
        // Vector used for computing B and C: Delta_p - (Delta_p, va) * va
        float3 C_vector = sub(deltaP, scaledAxis);

        float A = dot(A_vector, A_vector);
        float B = 2.0f * dot(A, C);
        float C = dot(C_vector, C_vector) - radiusSquared;

        float discriminant = B * B - 4.0f * A * C;

        if (discriminant >= 0.0f)
        {
            float oneOverTwoA = 0.5 / A;
            float delta = sqrtf(discriminant);
            float distance1 = (-B + delta) * oneOverTwoA;
            float distance2 = (-B - delta) * oneOverTwoA;

            float3 intersection1 = scale(distance1, direction);
            if (fabsf(percentageAlongLine3D(intersection1, cylinderPosition, normalizedCylinderAxis)) > halfLength - 1e-12f)
            {
                distance1 = nanf("");
            }

            if (!isnanf(distance1))
            {
                if (isnanf(dIntersection1) || fabsf(distance1 - dIntersection1) < 1e-12f)
                {
                    dIntersection1 = distance1;
                }
                else if (distance1 < dIntersection1)
                {
                    dIntersection2 = dIntersection1;
                    dIntersection1 = distance1;
                }
                else
                {
                    dIntersection2 = distance1;
                }
            }

            float3 intersection2 = scale(distance2, direction);
            if (fabsf(percentageAlongLine3D(intersection2, cylinderPosition, normalizedCylinderAxis)) > halfLength - 1e-12f)
            {
                distance2 = nanf("");
            }
            else if (fabsf(distance1 - distance2) < 1e-12f)
            {
                distance2 = nanf("");
            }

            if (!isnanf(distance2))
            {
                if (isnanf(dIntersection2))
                {
                    dIntersection1 = distance2;
                }
                else if (distance2 < dIntersection1)
                {
                    dIntersection2 = dIntersection1;
                    dIntersection1 = distance2;
                }
                else
                {
                    dIntersection2 = distance2;
                }
            }
        }
    }

    if (isnanf(dIntersection1))
        return INFINITY;
    if (isnanf(dIntersection2))
        return dIntersection1;

    return fminf(dIntersection1, dIntersection2);
}

__device__ float getDepthOfClosestCollisionOnCapsule(float3 point, float3 topCenter, float3 bottomCenter, float radius)
{
    float intersectionTolerance = 0.03;
    // This can be thought of as the minimum distance between collisions with both spheres and the cylinder
    float minDistanceToTop = getDepthOfClosestCollisionOnSphere(point, topCenter, radius + intersectionTolerance);
    float minDistanceToBottom = getDepthOfClosestCollisionOnSphere(point, bottomCenter, radius + intersectionTolerance);
    float minDistanceToCylinder = getDepthOfClosestCollisionOnCylinder(point, topCenter, bottomCenter, radius + intersectionTolerance);

    float closestCollision = fminf(fminf(minDistanceToTop, minDistanceToBottom), minDistanceToCylinder);
    return closestCollision;
}

__device__ bool isPointPastCapsule(float3 point, float3 topCenter, float3 bottomCenter, float radius)
{
    float tolerance = 0.05;
    float distanceToCapsule = getDepthOfClosestCollisionOnCapsule(point, topCenter, bottomCenter, radius);
    float distanceToPoint = length(point);

    return distanceToPoint > distanceToCapsule - tolerance;
}

__device__ float distanceSquared(float3 pointA, float3 pointB)
{
    float3 diff = sub(point, topCenter);
    return dot(diff, diff);
}

__device__ bool isPointInCapsule(float3 point,
                                 float3 topCenter,
                                 float3 bottomCenter,
                                 float radius)
{
    float tolerance = 0.1;
    float3 capsuleAxis = sub(bottomCenter, topCenter);
    float capsuleLengthSq = dot(capsuleAxis, capsuleAxis);
    float capsuleLength = sqrtf(capsuleLengthSq);

    if (capsuleLength < 1e-6f)
    {   // The capsule isn't a capsule here. Here, it's just a sphere.
        float distSq = distanceSquared(point, topCenter);
        return distSq <= (radius + tolerance) * (radius + tolerance);
    }

    // This computes the direction of the capsule axis
    float3 normalizedAxis = scale(1.0 / capsuleLength, capsuleAxis);
    float3 toPoint = sub(point, topCenter);

    float projection = dot(toPoint, normalizedAxis);
    projection = fmaxf(0.0f, fminf(projection, capsuleLength));

    // = topCenter + projection * normalizedAxis;
    float3 closestPointOnAxis = add(topCenter, scale(projection, normalizedAxis));

    float3 dist = sub(point, closestPointOnAxis);
    float distSq = dot(dist, dist);
    return distSq <= (radius + tolerance) * (radius + tolerance);
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

    if (x_index >= width || y_index >= height)
        return;

    unsigned short depthValue = *row(col(depthImage, x_index), depthImagePitch, y_index);

    // Convert from the depth value to the depth in meters.
    float depthInMeters = depthValue/1000.0f;
    // This is the collision point in the camera frame, using a projection model. The projection model has a focal length in x and y (fx and fy), and a center
    // pixel of the camera (cx and cy), and then just uses like triangles to compute the y and z coordinates.
    // The camera frame is defined as x forward and z up.
    float3 depthFramePoint = make_float3(depthInMeters,
                                          -(x_index - cx) / fx * depthInMeters,
                                         -(y_index - cy) / fy * depthInMeters);

    unsigned short *matrixRow = (unsigned short *)((char *)collisionMaskMap + y_index * pitchCollisionMaskMap) + x_index;
    *matrixRow = depthValue;

    for (int i = 0; i < numCollidables; ++i)
    {
        int index = i * numberOfAttributes;

        // Capsules are defined as a line segment that then has a fixed radius around it.
        // get the top point of the line segment in the camera frame.
        float3 topCenter = make_float3(collidableGeometryPointer[index],
                                       collidableGeometryPointer[index + 1],
                                       collidableGeometryPointer[index + 2]);

        // get the bottom point of the line segment in teh camera frame.
        float3 bottomCenter = make_float3(collidableGeometryPointer[index + 3],
                                          collidableGeometryPointer[index + 4],
                                          collidableGeometryPointer[index + 5]);

        float radius = collidableGeometryPointer[index + 6];

        if (isPointInCapsule(depthFramePoint, topCenter, bottomCenter, radius) || isPointPastCapsule(depthFramePoint, topCenter, bottomCenter, radius))
        {
            unsigned short *matrixRow = (unsigned short *)((char *)collisionMaskMap + y_index * pitchCollisionMaskMap) + x_index;
            *matrixRow = 0;
            return; // No need to check other capsules if a collision is found
        }
    }
}