__device__ float dot(const float3 a, const float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 transformPoint3D32_2(float3 point, float3 rotationMatrixRow0, float3 rotationMatrixRow1, float3 rotationMatrixRow2, float3 translation)
{
    return make_float3(dot(rotationMatrixRow0, point) + translation.x, dot(rotationMatrixRow1, point) + translation.y,
                       dot(rotationMatrixRow2, point) + translation.z);
}

__device__ float clamp(float value, float minVal, float maxVal)
{
    return fminf(fmaxf(value, minVal), maxVal);
}

__device__ float length2D(float2 vec)
{
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

