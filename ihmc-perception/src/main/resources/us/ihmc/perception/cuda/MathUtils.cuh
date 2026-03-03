#ifndef MATH_UTILS
#define MATH_UTILS

const float EPSILON_F = 1e-6f;
const float EPSILON_D = 1e-6;
const float PI_F = 3.1415927f;

__device__ __forceinline__
float2 operator+(const float2& a, const float2& b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

__device__ __forceinline__
float2 operator-(const float2& a, const float2& b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}

__device__ __forceinline__
float2 operator*(const float2& a, const float scalar)
{
    return make_float2(a.x * scalar, a.y * scalar);
}

__device__ __forceinline__
float2 operator/(const float2& a, const float divisor)
{
    return make_float2(a.x / divisor, a.y / divisor);
}

__device__ __forceinline__
float3 operator+(const float3& a, const float3& b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ __forceinline__
float3 operator-(const float3& a, const float3& b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ __forceinline__
float3 operator*(const float3& a, const float scalar)
{
    return make_float3(a.x * scalar, a.y * scalar, a.z * scalar);
}

__device__ __forceinline__
float3 operator/(const float3& a, const float divisor)
{
    return make_float3(a.x / divisor, a.y / divisor, a.z / divisor);
}

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
__device__ float normSquared(float3 v)
{
    return dot(v, v);
}

__device__ float distanceSquared(float3 pointA, float3 pointB)
{
    return normSquared(sub(pointA, pointB));
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

    if (norm < EPSILON_F)
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

__device__ float2 transformToWorld(const float2 &coordinate, float frameYaw)
{
    float sinYaw = sin(frameYaw);
    float cosYaw = cos(frameYaw);
    return make_float2(coordinate.x * cosYaw - coordinate.y * sinYaw, coordinate.x * sinYaw + coordinate.y * cosYaw);
}

__device__ float2 transformFromWorld(const float2 &coordinate, float frameYaw)
{
    return transformToWorld(coordinate, -frameYaw);
}

/**
 * Solves the 3 dimensional linear system Ax=b for x, using closed-form solution by computing determinants.
 * Data is expected to be row major, or [row1, row2, row3]
 **/
__device__ bool solve3x3System_Determinants(const float* matrixA, const float* vectorB, float* solutionXToPack)
{
    double m00 = (double) matrixA[0];
    double m01 = (double) matrixA[1];
    double m02 = (double) matrixA[2];
    double m10 = (double) matrixA[3];
    double m11 = (double) matrixA[4];
    double m12 = (double) matrixA[5];
    double m20 = (double) matrixA[6];
    double m21 = (double) matrixA[7];
    double m22 = (double) matrixA[8];

    // compute the determinant
   double det = m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m02 * m11 * m20 - m01 * m10 * m22 - m00 * m12 * m21;
   if (fabs(det) < EPSILON_D)
        return false;

   double detMinor00 = m11 * m22 - m12 * m21;
   double detMinor01 = m10 * m22 - m12 * m20;
   double detMinor02 = m10 * m21 - m11 * m20;

   double detMinor10 = m01 * m22 - m02 * m21;
   double detMinor11 = m00 * m22 - m02 * m20;
   double detMinor12 = m00 * m21 - m01 * m20;

   double detMinor20 = m01 * m12 - m02 * m11;
   double detMinor21 = m00 * m12 - m02 * m10;
   double detMinor22 = m00 * m11 - m01 * m10;

   double solutionX0 = 0.0;
   solutionX0 += vectorB[0] * detMinor00 / det;
   solutionX0 += vectorB[1] * -detMinor10 / det;
   solutionX0 += vectorB[2] * detMinor20 / det;
   solutionXToPack[0] = (float) solutionX0;

   double solutionX1 = 0.0;
   solutionX1 += vectorB[0] * -detMinor01 / det;
   solutionX1 += vectorB[1] * detMinor11 / det;
   solutionX1 += vectorB[2] * -detMinor21 / det;
   solutionXToPack[1] = (float) solutionX1;

   double solutionX2 = 0.0;
   solutionX2 += vectorB[0] * detMinor02 / det;
   solutionX2 += vectorB[1] * -detMinor12 / det;
   solutionX2 += vectorB[2] * detMinor22 / det;
   solutionXToPack[2] = (float) solutionX2;

   return true;
}

/**
 * Solves the 3 dimensional linear system Ax=b for x, using Cholesky decomposition into LL^Tx = b (L is lower triangular matrix).
 * Data is expected to be row major, or [row1, row2, row3]
 **/
__device__ bool solve3x3System_Cholesky(const float* matrixA, const float* vectorB, float* solutionXToPack)
{
    // Lower triangular L
    float L00, L10, L11, L20, L21, L22;

    // Factorization L * L^T = A
    L00 = sqrtf(matrixA[0]);
    if (L00 < EPSILON_F)
        return false;

    L10 = matrixA[3] / L00;
    L20 = matrixA[6] / L00;

    float t11 = matrixA[4] - L10 * L10;
    if (t11 < EPSILON_F)
        return false;
    L11 = sqrtf(t11);

    L21 = (matrixA[7] - L20 * L10) / L11;

    float t22 = matrixA[8] - L20 * L20 - L21 * L21;
    if (t22 < EPSILON_F)
        return false;
    L22 = sqrtf(t22);

    // Forward substitution: L * y = b
    float y0 = vectorB[0] / L00;
    float y1 = (vectorB[1] - L10 * y0) / L11;
    float y2 = (vectorB[2] - L20 * y0 - L21 * y1) / L22;

    // Backward substitution: L^T * x = y
    solutionXToPack[2] = y2 / L22;
    solutionXToPack[1] = (y1 - L21 * solutionXToPack[2]) / L11;
    solutionXToPack[0] = (y0 - L10 * solutionXToPack[1] - L20 * solutionXToPack[2]) / L00;

    return true;
}

/**
 * Common data used when computing best-fit planes on a local sample of a point-cloud.
 */
struct CovarianceData
{
    float sum_xx; // Σ x_i^2
    float sum_xy; // Σ x_i y_i
    float sum_xz; // Σ x_i z_i
    float sum_yy; // Σ y_i^2
    float sum_yz; // Σ y_i z_i
    float sum_zz; // Σ z_i^2

    float sum_x; // Σ x_i
    float sum_y; // Σ y_i
    float sum_z; // Σ z_i

    float numberOfPoints;

    __device__ void registerPoint(float x, float y, float z)
    {
        sum_xx += x * x;
        sum_xy += x * y;
        sum_xz += x * z;
        sum_yy += y * y;
        sum_yz += y * z;
        sum_zz += z * z;

        sum_x += x;
        sum_y += y;
        sum_z += z;

        numberOfPoints += 1.0f;
    }

    /**
     * Packs the covariance data in a 3x3 matrix A and 3x1 vector b, where Ax=b represents the best-fit plane.
     * Data is row major.
     */
    __device__ void packLinearSystem(float matrixAToPack[9], float matrixBToPack[3])
    {
        matrixAToPack[0] = sum_xx;
        matrixAToPack[1] = sum_xy;
        matrixAToPack[2] = sum_x;
        matrixAToPack[3] = sum_xy;
        matrixAToPack[4] = sum_yy;
        matrixAToPack[5] = sum_y;
        matrixAToPack[6] = sum_x;
        matrixAToPack[7] = sum_y;
        matrixAToPack[8] = numberOfPoints;

        matrixBToPack[0] = -sum_xz;
        matrixBToPack[1] = -sum_yz;
        matrixBToPack[2] = -sum_z;
    }
};

/**
 * Compute the mean squared error of a plane fit using least-squares sums. The plane is defined as: z = A*x + B*y + C
 */
__device__ float computePlaneSquaredError(float* coefficients, CovarianceData& covarianceData)
{
    float A = coefficients[0];
    float B = coefficients[1];
    float C = coefficients[2];

    // ---- Terms from x contributions ----
    float term_xx = A * A * covarianceData.sum_xx;
    float term_xy = 2.0f * A * B * covarianceData.sum_xy;
    float term_xz = 2.0f * A * covarianceData.sum_xz;
    float term_xC = 2.0f * A * C * covarianceData.sum_x;

    // ---- Terms from y contributions ----
    float term_yy = B * B * covarianceData.sum_yy;
    float term_yz = 2.0f * B * covarianceData.sum_yz;
    float term_yC = 2.0f * B * C * covarianceData.sum_y;

    // ---- Terms from constant C ----
    float term_CC = C * C * covarianceData.numberOfPoints;
    float term_Cz = 2.0f * C * covarianceData.sum_z;

    // ---- Terms independent of plane coefficients ----
    float term_zz = covarianceData.sum_zz;

    // ---- Sum all contributions to get total squared error ----
    float total_squared_error = term_xx + term_xy + term_xz + term_xC
                              + term_yy + term_yz + term_yC
                              + term_CC + term_Cz
                              + term_zz;

    // ---- Normalize by number of points to get mean squared error ----
    float mean_squared_error = total_squared_error / covarianceData.numberOfPoints;

    return mean_squared_error;
}

__device__ bool solveForPlaneNormal2x2(CovarianceData& covarianceData, float3& normal)
{
    float sum_xx = covarianceData.sum_xx;
    float sum_yy = covarianceData.sum_yy;
    float sum_xy = covarianceData.sum_xy;
    float sum_xz = covarianceData.sum_xz;
    float sum_yz = covarianceData.sum_yz;

    float sum_x = covarianceData.sum_x;
    float sum_y = covarianceData.sum_y;
    float sum_z = covarianceData.sum_z;

    // regularization
    sum_xx += EPSILON_F;
    sum_yy += EPSILON_F;

    float det = sum_xx * sum_yy - sum_xy * sum_xy;

    // degenerate neighborhood
    if (fabs(det) < 1e-8)
        return false;

    float invDet = 1.0f / det;

    float A = ( sum_yy * sum_xz - sum_xy * sum_yz) * invDet;
    float B = (-sum_xy * sum_xz + sum_xx * sum_yz) * invDet;

    // Plane: Ax + By - z + C = 0
    normal = make_float3(A, B, -1.0f);

    // Normalize normal
    float norm = rsqrtf(A*A + B*B + 1.0f);
    normal.x *= norm;
    normal.y *= norm;
    normal.z *= norm;

    return true;
}

/**
 * Computes the coefficients A, B, and C of a plane defined as: z = A*x + B*y + C given the local covariance data.
 * The linear system is solved using matrix determinants.
 */
__device__ float solveForPlaneCoefficients3x3_Determinants(CovarianceData& covarianceData, float* coefficients)
{
    float covarianceMatrix[9];
    float covarianceVector[3];
    covarianceData.packLinearSystem(covarianceMatrix, covarianceVector);

    if (!solve3x3System_Determinants(covarianceMatrix, covarianceVector, coefficients))
        return 1.0f / 0.0f;

    return computePlaneSquaredError(coefficients, covarianceData);
}

/**
 * Computes the coefficients A, B, and C of a plane defined as: z = A*x + B*y + C given the local covariance data.
 * The linear system is solved using Cholesky decomposition.
 */
__device__ float solveForPlaneCoefficients3x3_Cholesky(CovarianceData& covarianceData, float* coefficients)
{
    float covarianceMatrix[9];
    float covarianceVector[3];
    covarianceData.packLinearSystem(covarianceMatrix, covarianceVector);

    if (!solve3x3System_Cholesky(covarianceMatrix, covarianceVector, coefficients))
        return 1.0f / 0.0f;

    return computePlaneSquaredError(coefficients, covarianceData);
}

#endif // MATH_UTILS
