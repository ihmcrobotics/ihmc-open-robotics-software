#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

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
        float3 diff = sub(point, topCenter);
        float distSq = dot(diff, diff);
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

        if (isPointInCapsule(depthFramePoint, topCenter, bottomCenter, radius))
        {
            unsigned short *matrixRow = (unsigned short *)((char *)collisionMaskMap + y_index * pitchCollisionMaskMap) + x_index;
            *matrixRow = 0;
            return; // No need to check other capsules if a collision is found
        }
    }
}