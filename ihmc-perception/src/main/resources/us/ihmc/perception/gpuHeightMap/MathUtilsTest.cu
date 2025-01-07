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