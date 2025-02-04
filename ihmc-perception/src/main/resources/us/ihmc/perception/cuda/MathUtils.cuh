#ifndef MATH_UTILS
#define MATH_UTILS

const float PI_F = 3.1415927f;

__device__ float dot(const float3 a, const float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float dot2D(const float2 a, const float2 b)
{
    return a.x * b.x + a.y * b.y;
}

__device__ float3 transformPoint3D(float3 point, const float* transform)
{
    return make_float3(dot(make_float3(transform[0], transform[1], transform[2]), point) + transform[3],
                       dot(make_float3(transform[4], transform[5], transform[6]), point) + transform[7],
                       dot(make_float3(transform[8], transform[9], transform[10]), point) + transform[11]);
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

__device__ float3 normalize(const float3& vec)
{
    float magnitude = sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);

    // Prevent division by zero
    if (magnitude > 0.0f)
    {
        return make_float3(vec.x / magnitude, vec.y / magnitude, vec.z / magnitude);
    }
    else
    {
        return make_float3(0.0f, 0.0f, 0.0f);
    }
}

/**
 * Returns a 9 element array that is the inverse of a 9 element argument. The data is expected to be row major,
 * or [row1, row2, row3];
 **/
__device__ float* invert3x3Matrix(float* matrix)
{
    float m00 = matrix[0];
    float m01 = matrix[1];
    float m02 = matrix[2];
    float m10 = matrix[3];
    float m11 = matrix[4];
    float m12 = matrix[5];
    float m20 = matrix[6];
    float m21 = matrix[7];
    float m22 = matrix[8];

    // compute the determinant
   float det = m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m02 * m11 * m20 - m01 * m10 * m22 - m00 * m12 * m21;
   float ret[9];

   float detMinor00 = m11 * m22 - m12 * m21;
   float detMinor01 = m10 * m22 - m12 * m20;
   float detMinor02 = m10 * m21 - m11 * m20;

   float detMinor10 = m01 * m22 - m02 * m21;
   float detMinor11 = m00 * m22 - m02 * m20;
   float detMinor12 = m00 * m21 - m01 * m20;

   float detMinor20 = m01 * m12 - m02 * m11;
   float detMinor21 = m00 * m12 - m02 * m10;
   float detMinor22 = m00 * m11 - m01 * m10;

   ret[0] = detMinor00 / det;
   ret[1] = -detMinor10 / det;
   ret[2] = detMinor20 / det;

   ret[3] = -detMinor01 / det;
   ret[4] = detMinor11 / det;
   ret[5] = -detMinor21 / det;

   ret[6] = detMinor02 / det;
   ret[7] = -detMinor12 / det;
   ret[8] = detMinor22 / det;

   return ret;
}

__device__ void solveForPlaneCoefficients(float* covariance_matrix, float* z_variance_vector, float* coefficients)
{
    // Invert the 3x3 covariance matrix (this should be done on the device as well)
    float* inverse_covariance_matrix = invert3x3Matrix(covariance_matrix);  // Assuming this is a device function

    // Simple matrix multiplication: coefficients = inverse_covariance_matrix * z_variance_vector
    for (int row = 0; row < 3; row++)
    {
        coefficients[row] = 0.0f;  // Ensure the coefficients are reset before summing
        for (int col = 0; col < 3; col++)
        {
            coefficients[row] += inverse_covariance_matrix[row * 3 + col] * z_variance_vector[col];
        }
    }
}

#endif // MATH_UTILS
