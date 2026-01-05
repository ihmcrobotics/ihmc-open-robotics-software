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

__device__ float computeColorWeight(const float3 &ci, const float3 &cj, float sigma_c)
{
    float3 diff = sub(ci, cj);
    float dist2 = dot(diff, diff);
    float denom = 2.0f * sigma_c * sigma_c;
    return __expf(-dist2 / denom);
}

__device__ int gatherNeighbors(
    unsigned short *depthImage, size_t pitchDepthImage,
    unsigned char *coloredImage, size_t pitchColoredImage,
    int width, int height,
    int u, int v, int windowSize,
    float3 *points, float3 *colors, int maxNeighbors)
{
    int half = windowSize / 2;
    int count = 0;
    for (int dy = -half; dy <= half; ++dy)
    {
        int y = v + dy;
        if (y < 0 || y >= height) continue;

        unsigned short *depthRow = (unsigned short *)((char *)depthImage + y * pitchDepthImage);

        unsigned char *colorRow = (unsigned char *)((char *)coloredImage + y * pitchColoredImage);

        for (int dx = -half; dx <= half; ++dx)
        {
            int x = u + dx;
            if (x < 0 || x >= width) continue;

            unsigned short d = depthRow[x];
            if (d == 0) continue;

            points[count] = computeRay(x, y, d, fx, fy, cx, cy);

            float R = colorRow[3 * x + 0];
            float G = colorRow[3 * x + 1];
            float B = colorRow[3 * x + 2];
            colors[count] = make_float3(R, G, B);

            if (++count >= maxNeighbors) return count;
        }
    }
    return count;
}

__device__ float solveT(
    const float3 &pi, const float3 &ri,
    float3 *neighbors, float *weights, int count)
{
    float num = 0.0f;
    float den = 0.0f;

    for (int j = 0; j < count; ++j)
    {
        float3 pj = neighbors[j];
        float3 diff = sub(pi, pj);        // (p_i - p_j)
        float proj = dot(ri, diff);       // r_i^T (p_i - p_j)
        float w = weights[j];
        num += w * proj;
        den += w;
    }

    if (den < 1e-6f) return 0.0f;

    return -num / den;
}


// extern "C"
// __global__ void filterFlyingPoints(unsigned short *depthImage, size_t pitchDepthImage,
//                                    unsigned short *filteredImage, size_t pitchFilteredImage,
//                                    unsigned char *coloredImage, size_t pitchColoredImage
//                                    int width, int height,
//                                    int windowSizeInPixels, int ransacIterations, int minimumDepthValuesRequiredInWindow,
//                                    float angleThresholdInRadians, float normalAngleThreshold,
//                                    float fx, float fy, float cx, float cy)
// {
//     int u = blockIdx.x * blockDim.x + threadIdx.x;
//     int v = blockIdx.y * blockDim.y + threadIdx.y;
//
//     if (u >= width || v >= height)
//         return;
//
//     unsigned short *depthValue = (unsigned short *)((char *)depthImage + u * pitchDepthImage) + v;
//
//     if (*depthValue <= 0)
//     {
//         unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
//         filteredImageRow[v] = 0;
//         return;
//     }
//
//     float3 point = computeRay(u, v, *depthValue, fx, fy, cx, cy);
//     float3 ray = point;
//
//     float rayLength = length(ray);
//     if (rayLength < 1e-5f)
//     {
//         unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
//         filteredImageRow[v] = 0;
//         return;
//     }
//
//     sqrtDepth = colorizedRow[3 * x + 0];
//                 adjustmentA = colorizedRow[3 * x + 1];
//                 adjustmentB = colorizedRow[3 * x + 2];
//
//     ray = scale(1.0 / rayLength, ray);
//
//     float3 normal = computeNormalRANSAC(depthImage, pitchDepthImage,
//                                         width, height, u, v, fx, fy, cx, cy,
//                                         windowSizeInPixels, ransacIterations, minimumDepthValuesRequiredInWindow, normalAngleThreshold);
//
//     float angleBetweenVectors = dot(ray, normal);
//     angleBetweenVectors = fabsf(angleBetweenVectors);  // we want it close to 0, regardless of direction
//
//     unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
//     filteredImageRow[v] = (angleBetweenVectors < angleThresholdInRadians) ? *depthValue : 0;
// }

extern "C"
__global__ void filterFlyingPoints(
    unsigned short *depthImage, size_t pitchDepthImage,
    unsigned short *filteredImage, size_t pitchFilteredImage,
    unsigned char *coloredImage, size_t pitchColoredImage,
    int width, int height,
    int windowSizeInPixels, int ransacIterations, int minimumDepthValuesRequiredInWindow,
    float angleThresholdInRadians, float normalAngleThreshold,
    float fx, float fy, float cx, float cy,
    float sigma_c) // <-- new parameter for color weights
{
    int u = blockIdx.x * blockDim.x + threadIdx.x;
    int v = blockIdx.y * blockDim.y + threadIdx.y;
    if (u >= width || v >= height) return;

    unsigned short *depthRow = (unsigned short *)((char *)depthImage + v * pitchDepthImage);
    unsigned short d = depthRow[u];

    unsigned short *filteredRow = (unsigned short *)((char *)filteredImage + v * pitchFilteredImage);

    if (d == 0)
    {
        filteredRow[u] = 0;
        return;
    }

    float3 pi = computeRay(u, v, d, fx, fy, cx, cy);
    float3 ri = pi;
    float rayLength = length(ri);
    if (rayLength < 1e-5f)
    {
        filteredRow[u] = 0;
        return;
    }
    ri = scale(1.0f / rayLength, ri); // normalize

    // 1) normal-based FP detection
    float3 normal = computeNormalRANSAC(
        depthImage, pitchDepthImage,
        width, height, u, v, fx, fy, cx, cy,
        windowSizeInPixels, ransacIterations,
        minimumDepthValuesRequiredInWindow, normalAngleThreshold);

    float angleCos = fabsf(dot(ri, normal));
    float angle = acosf(fminf(fmaxf(angleCos, -1.0f), 1.0f)); // radians

    bool isFP = (angle >= angleThresholdInRadians); // or your condition

    if (!isFP)
    {
        // Keep native depth
        filteredRow[u] = d;
        return;
    }

    // 2) RGB-guided correction for flying pixels
    const int maxNeighbors = 64;
    float3 neighborPoints[maxNeighbors];
    float3 neighborColors[maxNeighbors];
    int neighborCount = gatherNeighbors(
        depthImage, pitchDepthImage,
        coloredImage, pitchColoredImage,
        width, height,
        u, v, windowSizeInPixels,
        neighborPoints, neighborColors, maxNeighbors);

    if (neighborCount < minimumDepthValuesRequiredInWindow)
    {
        // Could not correct reliably; option: drop pixel
        filteredRow[u] = 0;
        return;
    }

    // Color of current pixel
    unsigned char *colorRow = (unsigned char *)((char *)coloredImage + v * pitchColoredImage);
    float3 ci = make_float3(
        colorRow[3 * u + 0],
        colorRow[3 * u + 1],
        colorRow[3 * u + 2]);

    float weights[maxNeighbors];
    for (int j = 0; j < neighborCount; ++j)
        weights[j] = computeColorWeight(ci, neighborColors[j], sigma_c);

    float t = solveT(pi, ri, neighborPoints, weights, neighborCount);
    float3 p_corrected = add(pi, scale(t, ri));
    float correctedDepth = p_corrected.z; // or length(p_corrected);

    filteredRow[u] = (unsigned short)max(0.0f, min(correctedDepth, 65535.0f));
}
