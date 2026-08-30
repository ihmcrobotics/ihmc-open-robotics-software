#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

extern "C"
__global__ void extractPointCloud(unsigned short* depthImage,
                                  size_t pitch,
                                  int width,
                                  int height,
                                  float fx,
                                  float fy,
                                  float cx,
                                  float cy,
                                  float depthDiscretization,
                                  float minDepthMeters,
                                  float maxDepthMeters,
                                  float* depthToWorldTransform,
                                  float3* pointCloud,
                                  float* pointConfidence,        // parallel to pointCloud; may be null
                                  float confidenceReferenceRange,
                                  float confidenceFalloffExponent,
                                  float cameraTrustWeight,
                                  float* confidenceImage,        // ZED per-pixel confidence map (F32, aligned to depth); may be null
                                  size_t confidencePitch,        // row stride of confidenceImage in bytes
                                  int hasConfidenceImage,        // non-zero when confidenceImage is valid
                                  int* pointCloudSize)
{
    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short depthValue = *row(col(depthImage, x), pitch, y);
            if (depthValue == 0)
                continue;

            float depthInMeters = depthDiscretization * depthValue;
            if (depthInMeters < minDepthMeters || depthInMeters > maxDepthMeters)
                continue;

            float3 depthFramePoint = pixelDepthToPoint3D(x, y, depthInMeters, fx, fy, cx, cy);
            float3 worldFramePoint = transformPoint3D(depthFramePoint, depthToWorldTransform);

            float weight = depthConfidenceWeight(depthInMeters,
                                                 confidenceReferenceRange,
                                                 confidenceFalloffExponent,
                                                 cameraTrustWeight);

            // Fold in the ZED per-pixel stereo confidence (0/1 = best ... 100 = unreliable),
            // covering textureless/occluded regions that range alone can't capture.
            if (hasConfidenceImage)
            {
                float confidence = *row(col(confidenceImage, x), confidencePitch, y);
                float confidenceFactor = 1.0f - 0.01f * confidence;
                weight *= fminf(fmaxf(confidenceFactor, 0.0f), 1.0f);
            }

            int index = atomicAdd(pointCloudSize, 1);
            pointCloud[index] = worldFramePoint;
            if (pointConfidence != nullptr)
                pointConfidence[index] = weight;
        }
    }
}