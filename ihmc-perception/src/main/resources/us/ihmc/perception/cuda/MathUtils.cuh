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

__device__ float angle(float x1, float y1, float x2, float y2)
{
   float cosTheta = x1 * x2 + y1 * y2;
   float sinTheta = x1 * y2 - y1 * x2;
   return atan2(sinTheta, cosTheta);
}

// maps input value from range (lowerBound, upperBound) -> (0, 255)
__device__ unsigned char scaleAndCastToUnsignedChar(float val, float lowerBound, float upperBound)
{
   return static_cast<unsigned char>(255 * (val - lowerBound) / (upperBound - lowerBound));
}

__device__ float interpolate(float a, float b, float alpha)
{
    return (1.0f - alpha) * a + alpha * b;
}

__device__ float3 scale(float scalar, float3 point)
{
    float3 ret;
    ret.x = scalar * point.x;
    ret.y = scalar * point.y;
    ret.z = scalar * point.z;

    return ret;
}

__device__ float3 add(float3 pointA, float3 pointB)
{
    return make_float3(pointA.x + pointB.x, pointA.y + pointB.y, pointA.z + pointB.z);
}

__device__ float3 sub(float3 pointA, float3 pointB)
{
    return make_float3(pointA.x - pointB.x, pointA.y - pointB.y, pointA.z - pointB.z);
}

__device__ float3 transformPoint3D(float3 point, const float* transform)
{
    return make_float3(dot(make_float3(transform[0], transform[1], transform[2]), point) + transform[3],
                       dot(make_float3(transform[4], transform[5], transform[6]), point) + transform[7],
                       dot(make_float3(transform[8], transform[9], transform[10]), point) + transform[11]);
}

template  <typename T>
__device__ T clamp(const T& value, const T& min, const T& max)
{
    return min > value ? min : max < value ? max : value;
}

__device__ float length2D(float2 vec)
{
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

// Euclidean distance
__device__ float norm(float3 v)
{
    return dot(v, v);
}

__device__ float distanceSquared(float3 pointA, float3 pointB)
{
    return norm(sub(pointA, pointB));
}

__device__ float distance(float3 pointA, float3 pointB)
{
    return sqrtf(distanceSquared(pointA, pointB));
}

__device__ float length(float3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

__device__ float3 normalize(float3 v)
{
    float norm = sqrtf(dot(v, v));

    if (norm < 1e-6f)
        return make_float3(0, 0, 0);

    return make_float3(v.x / norm, v.y / norm, v.z / norm);
}

__device__ float3 cross3(const float3 &a, const float3 &b)
{
    return make_float3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

/**
 * Returns a 9 element array that is the inverse of a 9 element argument. The data is expected to be row major,
 * or [row1, row2, row3];
 **/
__device__ void invert3x3Matrix(double* matrix, double* result)
{
    double m00 = matrix[0];
    double m01 = matrix[1];
    double m02 = matrix[2];
    double m10 = matrix[3];
    double m11 = matrix[4];
    double m12 = matrix[5];
    double m20 = matrix[6];
    double m21 = matrix[7];
    double m22 = matrix[8];

    // compute the determinant
   double det = m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m02 * m11 * m20 - m01 * m10 * m22 - m00 * m12 * m21;

   double detMinor00 = m11 * m22 - m12 * m21;
   double detMinor01 = m10 * m22 - m12 * m20;
   double detMinor02 = m10 * m21 - m11 * m20;

   double detMinor10 = m01 * m22 - m02 * m21;
   double detMinor11 = m00 * m22 - m02 * m20;
   double detMinor12 = m00 * m21 - m01 * m20;

   double detMinor20 = m01 * m12 - m02 * m11;
   double detMinor21 = m00 * m12 - m02 * m10;
   double detMinor22 = m00 * m11 - m01 * m10;

   result[0] = detMinor00 / det;
   result[1] = -detMinor10 / det;
   result[2] = detMinor20 / det;

   result[3] = -detMinor01 / det;
   result[4] = detMinor11 / det;
   result[5] = -detMinor21 / det;

   result[6] = detMinor02 / det;
   result[7] = -detMinor12 / det;
   result[8] = detMinor22 / det;
}

__device__ double solveForPlaneCoefficients(double* covariance_matrix, double* z_variance_vector, double zz, double* coefficients)
{
    // Invert the 3x3 covariance matrix (this should be done on the device as well)
    double inverse_covariance_matrix[9];
    invert3x3Matrix(covariance_matrix, inverse_covariance_matrix);  // Assuming this is a device function

    // Simple matrix multiplication: coefficients = inverse_covariance_matrix * z_variance_vector
    for (int row = 0; row < 3; row++)
    {
        coefficients[row] = 0.0f;  // Ensure the coefficients are reset before summing
        for (int col = 0; col < 3; col++)
        {
            coefficients[row] += inverse_covariance_matrix[row * 3 + col] * z_variance_vector[col];
        }
    }

    // Compute squared error, from LeastSquaresPlaneFitter#fitPlaneToPoints
    double A = coefficients[0];
    double B = coefficients[1];
    double C = coefficients[2];

    double xx = covariance_matrix[0];
    double xy = covariance_matrix[1];
    double x = covariance_matrix[2];
    double yy = covariance_matrix[4];
    double y = covariance_matrix[5];
    double n = covariance_matrix[8];

    double xz = -z_variance_vector[0];
    double yz = -z_variance_vector[1];
    double z = -z_variance_vector[2];

    double squared_error = A*A * xx + 2 * A*B*xy + 2*A*xz + 2*A*C*x + B*B*yy + 2*B*yz + 2*B*C*y + zz + 2* C*z + n*C*C;
    return squared_error / n;
}

/**
 * Compute the mean squared error of a plane fit using least-squares sums.
 *
 * The plane is defined as: z = A*x + B*y + C
 * The inputs are precomputed sums from the sampled points:
 *   - sum_xx, sum_xy, sum_yy: sums of products of x and y
 *   - sum_xz, sum_yz, sum_zz: sums of products of x,y with z (relative to max)
 *   - sum_x, sum_y, sum_z: sums of x, y, z coordinates
 *   - num_points: total number of points used in the fit
 *
 * This makes the least-squares squared error calculation explicit
 * and easy to read/debug, using temporary variables for clarity.
 */
__device__ float computePlaneSquaredErrorVerbose(
    const float coefficients[3],  // [A, B, C]
    float sum_xx, float sum_xy, float sum_x,
    float sum_yy, float sum_y,
    float sum_xz, float sum_yz, float sum_z,
    float sum_zz,
    float num_points)
{
    float A = coefficients[0]; // Plane slope in x
    float B = coefficients[1]; // Plane slope in y
    float C = coefficients[2]; // Plane intercept

    // ---- Terms from x contributions ----
    float term_xx = A * A * sum_xx;       // A^2 * Σ(x_i^2)
    float term_xy = 2.0f * A * B * sum_xy; // 2 * A * B * Σ(x_i*y_i)
    float term_xz = 2.0f * A * sum_xz;     // 2 * A * Σ(x_i * z_i)
    float term_xC = 2.0f * A * C * sum_x;  // 2 * A * C * Σ(x_i)

    // ---- Terms from y contributions ----
    float term_yy = B * B * sum_yy;       // B^2 * Σ(y_i^2)
    float term_yz = 2.0f * B * sum_yz;     // 2 * B * Σ(y_i * z_i)
    float term_yC = 2.0f * B * C * sum_y;  // 2 * B * C * Σ(y_i)

    // ---- Terms from constant C ----
    float term_CC = C * C * num_points;    // C^2 * n
    float term_Cz = 2.0f * C * sum_z;      // 2 * C * Σ(z_i)

    // ---- Terms independent of plane coefficients ----
    float term_zz = sum_zz;                // Σ(z_i^2)

    // ---- Sum all contributions to get total squared error ----
    float total_squared_error = term_xx + term_xy + term_xz + term_xC
                              + term_yy + term_yz + term_yC
                              + term_CC + term_Cz
                              + term_zz;

    // ---- Normalize by number of points to get mean squared error ----
    float mean_squared_error = total_squared_error / num_points;

    return mean_squared_error;
}

#endif // MATH_UTILS
