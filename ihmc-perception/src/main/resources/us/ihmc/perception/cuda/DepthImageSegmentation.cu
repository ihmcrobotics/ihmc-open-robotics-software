#include "Utils.cu"
#include "PerceptionUtils.cu"
#include "MathUtils.cuh"

using namespace PerceptionUtils;

extern "C"
__global__ void segmentDepthImage(unsigned short* depthImage, size_t depthPitch,
                                  int width, int height,
                                  float depthFx, float depthFy,
                                  float depthCx, float depthCy,
                                  unsigned char* mask, size_t maskPitch,
                                  int maskWidth, int maskHeight,
                                  float maskFx, float maskFy,
                                  float maskCx, float maskCy,
                                  float depthDiscretization,
                                  float* depthToMaskTransform,
                                  unsigned short* outputImage, size_t outputPitch)
{
    int startX = Utils::getThreadCoordX();
    int strideX = Utils::getStrideX();

    int startY = Utils::getThreadCoordY();
    int strideY = Utils::getStrideY();

    for (int y = startY; y < height; y += strideY)
    {
        for (int x = startX; x < width; x += strideX)
        {
            unsigned short* outputCell = row(col(outputImage, x), outputPitch, y);

            // Get the depth
            unsigned short depthValue = *row(col(depthImage, x), depthPitch, y);
            if (depthValue == 0)
            {
                *outputCell = 0;
                continue;
            }

            float depthInMeters = depthValue * depthDiscretization;

            // Transform it to a point in mask frame
            float3 depthFramePoint = make_float3(depthInMeters,
                                                 -(x - depthCx) / depthFx * depthInMeters,
                                                 -(y - depthCy) / depthFy * depthInMeters);
            float3 maskFramePoint = transformPoint3D(depthFramePoint, depthToMaskTransform);

            // Find the mask's column and row corresponding to the depth image column and row
            float yaw = -angle(1.0f, 0.0f, maskFramePoint.x, maskFramePoint.y);
            int maskCol = __float2int_rn(maskCx + maskFx * tan(yaw));
            maskCol = clamp(maskCol, 0, maskWidth - 1);

            float pitch = -angle(1.0f, 0.0f, maskFramePoint.x, maskFramePoint.z);
            int maskRow = __float2int_rn(maskCy + maskFy * tan(pitch));
            maskRow = clamp(maskRow, 0, maskHeight - 1);

            // Check whether the mask includes or excludes the depth pixel
            unsigned char maskValue = *row(col(mask, maskCol), maskPitch, maskRow);
            if (maskValue == 0)
                *outputCell = 0;
            else
                *outputCell = depthValue;
        }
    }
}