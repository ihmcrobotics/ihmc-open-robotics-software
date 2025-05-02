__device__ float3 operator+(const float3 &a, const float3 &b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ float3 operator-(const float3 &a, const float3 &b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ float dot(const float3 &a, const float3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 cross(const float3 &a, const float3 &b)
{
    return make_float3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

__device__ float3 normalize(const float3 &v)
{
    float len = sqrtf(dot(v, v));
    if (len > 1e-5f)
        return make_float3(v.x / len, v.y / len, v.z / len);
    else
        return make_float3(0.0f, 0.0f, 0.0f);
}

__device__ float3 get3D(unsigned short* depthImage, size_t pitch, int x, int y,
                        int width, int height, float fx, float fy, float cx, float cy)
{
    if (x < 0 || x >= width || y < 0 || y >= height)
        return make_float3(0, 0, 0);

    unsigned short depth = *((unsigned short*)((char*)depthImage + y * pitch) + x);
    if (depth == 0)
        return make_float3(0, 0, 0);

    float d = depth * 0.001f; // Convert mm to meters
    return make_float3((x - cx) * d / fx,
                       (y - cy) * d / fy,
                       d);
}

extern "C" __global__
void computeNormalsAndFilterFlyingPoints(unsigned short* depthImage, size_t depthImagePitch,
                                         unsigned short* collisionMask, size_t collisionMaskPitch,
                                         int width, int height,
                                         float fx, float fy, float cx, float cy,
                                         float cosAngleThreshold, float depthThreshold,
                                         float normalViewThreshold, float depthVarianceThreshold)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height)
        return;

    float3 p     = get3D(depthImage, depthImagePitch, x,     y,     width, height, fx, fy, cx, cy);
    float3 px1   = get3D(depthImage, depthImagePitch, x + 1, y,     width, height, fx, fy, cx, cy);
    float3 py1   = get3D(depthImage, depthImagePitch, x,     y + 1, width, height, fx, fy, cx, cy);
    float3 pxm1  = get3D(depthImage, depthImagePitch, x - 1, y,     width, height, fx, fy, cx, cy);
    float3 pym1  = get3D(depthImage, depthImagePitch, x,     y - 1, width, height, fx, fy, cx, cy);

    float3 dx1 = make_float3(px1.x - p.x, px1.y - p.y, px1.z - p.z);
    float3 dy1 = make_float3(py1.x - p.x, py1.y - p.y, py1.z - p.z);
    float3 dx2 = make_float3(p.x - pxm1.x, p.y - pxm1.y, p.z - pxm1.z);
    float3 dy2 = make_float3(p.x - pym1.x, p.y - pym1.y, p.z - pym1.z);

    float3 n1 = normalize(cross(dx1, dy1));
    float3 n2 = normalize(cross(dx2, dy2));
    float3 normal = normalize(n1 + n2);

    float dotNeighbor = dot(n1, n2);

    float dzx = fabsf(px1.z - pxm1.z);
    float dzy = fabsf(py1.z - pym1.z);

    bool badNormal = dotNeighbor < cosAngleThreshold;
    bool badDepth  = false;//dzx > depthThreshold || dzy > depthThreshold;

    // --- New test: normal vs viewing ray ---
    float3 viewRay = normalize(p);
    float dotViewNormal = fabsf(dot(normal, viewRay));
    bool badViewNormal = dotViewNormal < normalViewThreshold;

    // --- New test: depth variance in 3x3 neighborhood ---
    float depthSum = 0.0f;
    float depthSqSum = 0.0f;
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            float3 q = get3D(depthImage, depthImagePitch, x + dx, y + dy, width, height, fx, fy, cx, cy);
            if (q.z > 0.001f)
            {
                depthSum += q.z;
                depthSqSum += q.z * q.z;
                ++count;
            }
        }
    }

    bool highDepthVariance = false;
    if (count > 0)
    {
        float mean = depthSum / count;
        float variance = (depthSqSum / count) - (mean * mean);
        highDepthVariance = variance > (depthVarianceThreshold * depthVarianceThreshold);
    }

    unsigned short* inputPtr = (unsigned short*)((char*)depthImage + y * depthImagePitch) + x;
    unsigned short* maskPtr = (unsigned short*)((char*)collisionMask + y * collisionMaskPitch) + x;
    *maskPtr = (badViewNormal) ? 0 : *inputPtr;
}