#include "MathUtils.cuh"

__device__ float3 computeRay(int u, int v, float depth, float fx, float fy, float cx, float cy)
{
    float x = (u - cx) * depth / fx;
    float y = (v - cy) * depth / fy;
    float z = depth;

    return make_float3(x, y, z);
}

__device__ float3 computeNormalRANSAC(unsigned short *depthImage, size_t pitchDepthImage,
                                      int width, int height,
                                      int u, int v, float fx, float fy, float cx, float cy,
                                      int windowSizeInPixels, int ransacIterations, int minimumDepthValuesRequiredInWindow, float normalAngleThreshold)
{
    const int halfWindow = windowSizeInPixels / 2;
    const int maxSamples = 32;

    float3 points[maxSamples];
    int count = 0;

    // Gather neighborhood points
    for (int dy = -halfWindow; dy <= halfWindow; ++dy)
    {
        for (int dx = -halfWindow; dx <= halfWindow; ++dx)
        {
            int x = u + dx;
            int y = v + dy;
            if (x < 0 || x >= width || y < 0 || y >= height)
                continue;

            unsigned short *depthValue = (unsigned short *)((char *)depthImage + x * pitchDepthImage) + y;

            if (*depthValue <= 0)
                continue;


            points[count++] = computeRay(x, y, *depthValue, fx, fy, cx, cy);

            if (count >= maxSamples)
                break;
        }

        if (count >= maxSamples)
            break;
    }

    if (count < minimumDepthValuesRequiredInWindow)
        return make_float3(0, 0, 0);

    float3 bestNormal = make_float3(0, 0, 0);
    int bestInliers = 0;

    for (int i = 0; i < ransacIterations; ++i)
    {
        int i1 = i % count;
        int i2 = (i + 1) % count;
        int i3 = (i + 2) % count;

        float3 p1 = points[i1];
        float3 p2 = points[i2];
        float3 p3 = points[i3];

        float3 v1 = sub(p2, p1);
        float3 v2 = sub(p3, p1);
        float3 normal = normalize(cross3(v1, v2));

        if (norm(normal) < 1e-5f)
            continue;

        // Count inliers: points whose normal agrees with this normal
        int inliers = 0;
        for (int j = 0; j < count; ++j)
        {
            float3 pj = points[j];
            float3 vj = sub(pj, p1);
            float len = length(vj);

            if (len < 1e-8f)
                continue;

            float angle = fabsf(dot(normal, normalize(vj)));
            float angleInRadians = acosf(angle);  // in radians

            if (angleInRadians < normalAngleThreshold)
                inliers++;
        }

        if (inliers > bestInliers)
        {
            bestInliers = inliers;
            bestNormal = normal;
        }
    }

    return bestNormal;
}

extern "C"
__global__ void filterFlyingPoints(unsigned short *depthImage, size_t pitchDepthImage,
                                   unsigned short *filteredImage, size_t pitchFilteredImage,
                                   int width, int height,
                                   int windowSizeInPixels, int ransacIterations, int minimumDepthValuesRequiredInWindow,
                                   float angleThresholdInRadians, float normalAngleThreshold,
                                   float fx, float fy, float cx, float cy)
{
    int u = blockIdx.x * blockDim.x + threadIdx.x;
    int v = blockIdx.y * blockDim.y + threadIdx.y;

    if (u >= width || v >= height)
        return;

    unsigned short *depthValue = (unsigned short *)((char *)depthImage + u * pitchDepthImage) + v;

    if (*depthValue <= 0)
    {
        unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
        filteredImageRow[v] = 0;
        return;
    }

    float3 point = computeRay(u, v, *depthValue, fx, fy, cx, cy);
    float3 ray = point;

    float rayLength = length(ray);
    if (rayLength < 1e-5f)
    {
        unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
        filteredImageRow[v] = 0;
        return;
    }

    ray = scale(1.0 / rayLength, ray);

    float3 normal = computeNormalRANSAC(depthImage, pitchDepthImage,
                                        width, height, u, v, fx, fy, cx, cy,
                                        windowSizeInPixels, ransacIterations, minimumDepthValuesRequiredInWindow, normalAngleThreshold);

    float angleBetweenVectors = dot(ray, normal);
    angleBetweenVectors = fabsf(angleBetweenVectors);  // we want it close to 0, regardless of direction

    unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
    filteredImageRow[v] = (angleBetweenVectors < angleThresholdInRadians) ? *depthValue : 0;
}