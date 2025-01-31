#include "MathUtils.cuh"

extern "C"
__global__ void test_math_utils_dot_product(float ax,
                                            float ay,
                                            float az,
                                            float bx,
                                            float by,
                                            float bz,
                                            float* result)
{
    printf("A (%f, %f, %f) and B (%f, %f, %f)\n", ax, ay, az, bx, by, bz);
    int index = blockIdx.x * blockDim.x + threadIdx.x;

    float3 aVector = make_float3(ax, ay, az);
    float3 bVector = make_float3(bx, by, bz);

    result[index] = dot(aVector, bVector);
}

extern "C"
__global__ void test_math_utils_clamp(float value,
                                      float minValue,
                                      float maxValue,
                                      float* result)
{
    printf("Value: %f, MaxValue: %f, MinValue: %f", value, minValue, maxValue);

    *result = clamp(value, minValue, maxValue);
}

extern "C"
__global__ void test_math_utils_transform_point(float px, float py, float pz,
                                                float r0x, float r0y, float r0z,
                                                float r1x, float r1y, float r1z,
                                                float r2x, float r2y, float r2z,
                                                float tx, float ty, float tz,
                                                float* result)
{
    printf("Point: (%f, %f, %f)\n", px, py, pz);
    printf("Rotation Matrix Rows:\n");
    printf("Row 0: (%f, %f, %f)\n", r0x, r0y, r0z);
    printf("Row 1: (%f, %f, %f)\n", r1x, r1y, r1z);
    printf("Row 2: (%f, %f, %f)\n", r2x, r2y, r2z);
    printf("Translation: (%f, %f, %f)\n", tx, ty, tz);

    float3 point = make_float3(px, py, pz);
    float3 row0 = make_float3(r0x, r0y, r0z);
    float3 row1 = make_float3(r1x, r1y, r1z);
    float3 row2 = make_float3(r2x, r2y, r2z);
    float3 translation = make_float3(tx, ty, tz);

    float3 transformedPoint = transformPoint3D32_2(point, row0, row1, row2, translation);

    int index = blockIdx.x * blockDim.x + threadIdx.x;
    result[index * 3 + 0] = transformedPoint.x;
    result[index * 3 + 1] = transformedPoint.y;
    result[index * 3 + 2] = transformedPoint.z;

    printf("Transformed Point: (%f, %f, %f)\n", transformedPoint.x, transformedPoint.y, transformedPoint.z);
}