__device__ float dot3(const float3 &a, const float3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 cross3(const float3 &a, const float3 &b)
{
    return make_float3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

__device__ float3 computeRay(int u, int v, float depth, float fx, float fy, float cx, float cy)
{
    float x = (u - cx) * depth / fx;
    float y = (v - cy) * depth / fy;
    float z = depth;

    return make_float3(x, y, z);
}

__device__ float3 normalize3(float3 v)
{
    float norm = sqrtf(dot3(v, v));

    if (norm < 1e-5f)
        return make_float3(0, 0, 0);

    return make_float3(v.x / norm, v.y / norm, v.z / norm);
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

        float3 v1 = make_float3(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        float3 v2 = make_float3(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        float3 normal = normalize3(cross3(v1, v2));

        if (dot3(normal, normal) < 1e-5f)
            continue;

        // Count inliers: points whose normal agrees with this normal
        int inliers = 0;
        for (int j = 0; j < count; ++j)
        {
            float3 pj = points[j];
            float3 vj = make_float3(pj.x - p1.x, pj.y - p1.y, pj.z - p1.z);
            float len = sqrtf(dot3(vj, vj));

            if (len < 1e-8f)
                continue;

            float dot = fabsf(dot3(normal, normalize3(vj)));
            float angle = acosf(dot);  // in radians

            if (angle < normalAngleThreshold)
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

    float rayNorm = sqrtf(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
    if (rayNorm < 1e-5f)
    {
        unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
        filteredImageRow[v] = 0;
        return;
    }

    ray.x /= rayNorm;
    ray.y /= rayNorm;
    ray.z /= rayNorm;

    float3 normal = computeNormalRANSAC(depthImage, pitchDepthImage,
                                        width, height, u, v, fx, fy, cx, cy,
                                        windowSizeInPixels, ransacIterations, minimumDepthValuesRequiredInWindow, normalAngleThreshold);

    float dot = dot3(ray, normal);
    dot = fabsf(dot);  // we want it close to 0, regardless of direction

    unsigned short *filteredImageRow = (unsigned short *)((char *)filteredImage + u * pitchFilteredImage);
    filteredImageRow[v] = (dot < angleThresholdInRadians) ? *depthValue : 0;
}