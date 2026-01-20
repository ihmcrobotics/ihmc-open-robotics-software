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

    float transform[12] =
    {
        r0x, r0y, r0z, tx,
        r1x, r1y, r1z, ty,
        r2x, r2y, r2z, tz
    };

    float3 transformedPoint = transformPoint3D(point, transform);

    int index = blockIdx.x * blockDim.x + threadIdx.x;
    result[index * 3 + 0] = transformedPoint.x;
    result[index * 3 + 1] = transformedPoint.y;
    result[index * 3 + 2] = transformedPoint.z;

    printf("Transformed Point: (%f, %f, %f)\n", transformedPoint.x, transformedPoint.y, transformedPoint.z);
}

extern "C"
__global__ void test_math_utils_transform_point_2(float px, float py, float pz, float* transformMatrix, float3& result)
{
    float3 transformedPoint = transformPoint3D(make_float3(px, py, pz), transformMatrix);

    result.x = transformedPoint.x;
    result.y = transformedPoint.y;
    result.z = transformedPoint.z;

    printf("Transformed Point (we are on the GPU rn): (%f, %f, %f)\n", result.x, result.y, result.z);
}

extern "C"
__global__ void test_best_fit_plane(float* points, int number_of_points, float* point_result, float* normal_result, float* squared_error_result, int test_index)
{
    CovarianceData covarianceData = {};

    for (int i = 0; i < number_of_points; i++)
    {
        float px = points[3 * i];
        float py = points[3 * i + 1];
        float pz = points[3 * i + 2];

        covarianceData.numberOfPoints += 1;
        covarianceData.sum_x += px;
        covarianceData.sum_y += py;
        covarianceData.sum_z += pz;
        covarianceData.sum_xx += px * px;
        covarianceData.sum_xy += px * py;
        covarianceData.sum_xz += px * pz;
        covarianceData.sum_yy += py * py;
        covarianceData.sum_yz += py * pz;
        covarianceData.sum_zz += pz * pz;
    }

    float coefficients[3] = {0.0f, 0.0f, 0.0f};

    // alternate testing cholesky and determinant
    float squared_error;
    if (test_index % 2 == 0)
    {
        squared_error = solveForPlaneCoefficients3x3_Determinants(covarianceData, coefficients);
    }
    else
    {
        squared_error = solveForPlaneCoefficients3x3_Cholesky(covarianceData, coefficients);
    }

    squared_error_result[0] = squared_error;

    point_result[0] = covarianceData.sum_x / number_of_points;
    point_result[1] = covarianceData.sum_y / number_of_points;
    point_result[2] = -coefficients[0] * point_result[0] - coefficients[1] * point_result[1] - coefficients[2];

    normal_result[0] = coefficients[0];
    normal_result[1] = coefficients[1];
    normal_result[2] = 1.0f;
}