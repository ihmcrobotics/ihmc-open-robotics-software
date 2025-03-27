#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

__device__ bool isPointInCapsule(float3 point,
                                 float3 topCenter,
                                 float3 bottomCenter,
                                 float radius) {
    float3 capsuleAxis = bottomCenter - topCenter;
    float capsuleLengthSq = dot(capsuleAxis, capsuleAxis);
    float capsuleLength = sqrtf(capsuleLengthSq);

    if (capsuleLength < 1e-6f) {
        float distSq = dot(point - topCenter, point - topCenter);
        return distSq <= radius * radius;
    }

    float3 normalizedAxis = capsuleAxis / capsuleLength;
    float3 toPoint = point - topCenter;
    float projection = dot(toPoint, normalizedAxis);
    projection = fmaxf(0.0f, fminf(projection, capsuleLength));
    float3 closestPointOnAxis = topCenter + projection * normalizedAxis;
    float distSq = dot(point - closestPointOnAxis, point - closestPointOnAxis);
    return distSq <= radius * radius;
}

extern "C"
__global__ void checkBodyCollision(unsigned short* depthImage,
                                  size_t pitch,
                                  int width,
                                  int height,
                                  float fx,
                                  float fy,
                                  float cx,
                                  float cy,
                                  float* collidableGeometry,
                                  int numCollidables,
                                  unsigned char* collisionMask) {
    int x = Utils::getThreadCoordX();
    int y = Utils::getThreadCoordY();
    int idx = y * width + x;

    if (x >= width || y >= height)
        return;

    unsigned short depthValue = *row(col(depthImage, x), pitch, y);
    if (depthValue == 0) {
        collisionMask[idx] = 0;
        return;
    }

    float depthInMeters = depthDiscretization * depthValue;
    float3 depthFramePoint = make_float3(depthInMeters,
                                         -(x - cx) / fx * depthInMeters,
                                         -(y - cy) / fy * depthInMeters);

    collisionMask[idx] = 0; // Initialize to 0

    for (int i = 0; i < numCollidables; ++i) {
        float3 topCenter = make_float3(collidableGeometry[i * 7 + 0],
                                         collidableGeometry[i * 7 + 1],
                                         collidableGeometry[i * 7 + 2]);
        float3 bottomCenter = make_float3(collidableGeometry[i * 7 + 3],
                                            collidableGeometry[i * 7 + 4],
                                            collidableGeometry[i * 7 + 5]);
        float radius = collidableGeometry[i * 7 + 6];

        if (isPointInCapsule(depthFramePoint, topCenter, bottomCenter, radius)) {
            collisionMask[idx] = 1;
            return; // No need to check other capsules if a collision is found
        }
    }
}