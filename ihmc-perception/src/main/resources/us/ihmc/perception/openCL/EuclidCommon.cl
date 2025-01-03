#define TRANSLATION_X 0
#define TRANSLATION_Y 1
#define TRANSLATION_Z 2
#define ROTATION_MATRIX_M00 3
#define ROTATION_MATRIX_M01 4
#define ROTATION_MATRIX_M02 5
#define ROTATION_MATRIX_M10 6
#define ROTATION_MATRIX_M11 7
#define ROTATION_MATRIX_M12 8
#define ROTATION_MATRIX_M20 9
#define ROTATION_MATRIX_M21 10
#define ROTATION_MATRIX_M22 11

float2 apply2DRotationToVector2D(float2 vector, float cosH, float sinH)
{
   float dxLocal = cosH * vector.x - sinH * vector.y;
   float dyLocal = sinH * vector.x + cosH * vector.y;

   return (float2) (dxLocal, dyLocal);
}

float2 applyYawRotationToVector2D(float2 vector, float yaw)
{
   float cY = cos(yaw);
   float sY = sin(yaw);

   return apply2DRotationToVector2D(vector, cY, sY);
}

float2 applyInverseYawRotationToVector2D(float2 vector, float yaw)
{
   float cY = cos(-yaw);
   float sY = sin(-yaw);

   return apply2DRotationToVector2D(vector, cY, sY);
}

float4 transform(float x,
                 float y,
                 float z,
                 float translationX,
                 float translationY,
                 float translationZ,
                 float rotationMatrixM00,
                 float rotationMatrixM01,
                 float rotationMatrixM02,
                 float rotationMatrixM10,
                 float rotationMatrixM11,
                 float rotationMatrixM12,
                 float rotationMatrixM20,
                 float rotationMatrixM21,
                 float rotationMatrixM22)
{
   float4 ret =
       (float4) (rotationMatrixM00 * x + rotationMatrixM01 * y + rotationMatrixM02 * z, rotationMatrixM10 * x + rotationMatrixM11 * y + rotationMatrixM12 * z,
                 rotationMatrixM20 * x + rotationMatrixM21 * y + rotationMatrixM22 * z, 0.0f);
   ret.x += translationX;
   ret.y += translationY;
   ret.z += translationZ;
   return ret;
}

float3 transformPoint3D32(float3 point, global float* transform)
{
   return (float3) (dot((float3) (transform[ROTATION_MATRIX_M00], transform[ROTATION_MATRIX_M01], transform[ROTATION_MATRIX_M02]), point) +
                        transform[TRANSLATION_X],
                    dot((float3) (transform[ROTATION_MATRIX_M10], transform[ROTATION_MATRIX_M11], transform[ROTATION_MATRIX_M12]), point) +
                        transform[TRANSLATION_Y],
                    dot((float3) (transform[ROTATION_MATRIX_M20], transform[ROTATION_MATRIX_M21], transform[ROTATION_MATRIX_M22]), point) +
                        transform[TRANSLATION_Z]);
}

float3 transformPoint3D32_2(float3 point, float3 rotationMatrixRow0, float3 rotationMatrixRow1, float3 rotationMatrixRow2, float3 translation)
{
   return (float3) (dot(rotationMatrixRow0, point) + translation.x, dot(rotationMatrixRow1, point) + translation.y,
                    dot(rotationMatrixRow2, point) + translation.z);
}

float16 prependYawRotation(float yaw, float16 matrixOriginal)
{
   double cYaw = cos(yaw);
   double sYaw = sin(yaw);
   double m00 = cYaw * matrixOriginal.s0 - sYaw * matrixOriginal.s3;
   double m01 = cYaw * matrixOriginal.s1 - sYaw * matrixOriginal.s4;
   double m02 = cYaw * matrixOriginal.s2 - sYaw * matrixOriginal.s5;
   double m10 = sYaw * matrixOriginal.s0 + cYaw * matrixOriginal.s3;
   double m11 = sYaw * matrixOriginal.s1 + cYaw * matrixOriginal.s4;
   double m12 = sYaw * matrixOriginal.s2 + cYaw * matrixOriginal.s5;
   double m20 = matrixOriginal.s6;
   double m21 = matrixOriginal.s7;
   double m22 = matrixOriginal.s8;
   matrixOriginal.s0 = m00;
   matrixOriginal.s1 = m01;
   matrixOriginal.s2 = m02;
   matrixOriginal.s3 = m10;
   matrixOriginal.s4 = m11;
   matrixOriginal.s5 = m12;
   matrixOriginal.s6 = m20;
   matrixOriginal.s7 = m21;
   matrixOriginal.s8 = m22;
   return matrixOriginal;
}

float16 appendYawRotation(float yaw, float16 matrixOriginal)
{
   double cYaw = cos(yaw);
   double sYaw = sin(yaw);
   double m00 = cYaw * matrixOriginal.s0 + sYaw * matrixOriginal.s1;
   double m01 = -sYaw * matrixOriginal.s0 + cYaw * matrixOriginal.s1;
   double m02 = matrixOriginal.s2;
   double m10 = cYaw * matrixOriginal.s4 + sYaw * matrixOriginal.s5;
   double m11 = -sYaw * matrixOriginal.s4 + cYaw * matrixOriginal.s5;
   double m12 = matrixOriginal.s6;
   double m20 = cYaw * matrixOriginal.s6 + sYaw * matrixOriginal.s7;
   double m21 = -sYaw * matrixOriginal.s6 + cYaw * matrixOriginal.s7;
   double m22 = matrixOriginal.s8;
   matrixOriginal.s0 = m00;
   matrixOriginal.s1 = m01;
   matrixOriginal.s2 = m02;
   matrixOriginal.s3 = m10;
   matrixOriginal.s4 = m11;
   matrixOriginal.s5 = m12;
   matrixOriginal.s6 = m20;
   matrixOriginal.s7 = m21;
   matrixOriginal.s8 = m22;
   return matrixOriginal;
}

float16 setToPitchOrientation(float pitch, float16 rotationMatrix)
{
   float sinPitch = sin(pitch);
   float cosPitch = cos(pitch);
   rotationMatrix.s0 = cosPitch;
   rotationMatrix.s1 = 0.0;
   rotationMatrix.s2 = sinPitch;
   rotationMatrix.s3 = 0.0;
   rotationMatrix.s4 = 1.0;
   rotationMatrix.s5 = 0.0;
   rotationMatrix.s6 = -sinPitch;
   rotationMatrix.s7 = 0.0;
   rotationMatrix.s8 = cosPitch;
   return rotationMatrix;
}

float16 newRotationMatrix()
{
   return (float16) (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // unused values
}

float normSquared2D(float x, float y)
{
   return x * x + y * y;
}

float norm2D(float x, float y)
{
   return sqrt(normSquared2D(x, y));
}

float normSquared(float x, float y, float z)
{
   return x * x + y * y + z * z;
}

float norm(float x, float y, float z)
{
   return sqrt(normSquared(x, y, z));
}

float angle(float x1, float y1, float x2, float y2)
{
   float cosTheta = x1 * x2 + y1 * y2;
   float sinTheta = x1 * y2 - y1 * x2;
   return atan2(sinTheta, cosTheta);
}

float angle3D(float x1, float y1, float z1, float x2, float y2, float z2)
{
   float crossX = y1 * z2 - z1 * y2;
   float crossY = z1 * x2 - x1 * z2;
   float crossZ = x1 * y2 - y1 * x2;

   float cosTheta = x1 * x2 + y1 * y2 + z1 * z2;
   float sinTheta = norm(crossX, crossY, crossZ);
   return atan2(sinTheta, cosTheta);
}

float distanceSquaredBetweenPoint3Ds(float firstPointX, float firstPointY, float firstPointZ, float secondPointX, float secondPointY, float secondPointZ)
{
   float deltaX = secondPointX - firstPointX;
   float deltaY = secondPointY - firstPointY;
   float deltaZ = secondPointZ - firstPointZ;
   return normSquared(deltaX, deltaY, deltaZ);
}

bool intervalContains(float value, float lowerEndpoint, float upperEndpoint)
{
   return value >= lowerEndpoint && value <= upperEndpoint;
}

double interpolate(double a, double b, double alpha)
{
   return (1.0 - alpha) * a + alpha * b;
}

/**
 * Returns the determinant of a 3x3 matrix that is represented as a 9 element row major matrix.
 **/
float determinant3x3Matrix(float* matrix)
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

   return m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m02 * m11 * m20 - m01 * m10 * m22 - m00 * m12 * m21;
}

/**
 * Returns a 9 element array that is the inverse of a 9 element argument. The data is expected to be row major,
 * or [row1, row2, row3];
 **/
float* invert3x3Matrix(float* matrix)
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

/**
 * Taking in a random number seed and a bits mask, returns a random integer and the modified seed.
 **/
uint2 nextRandom(uint seed, uint bits)
{
   long multiplier = 0x5DEECE66DL;
   long addend = 0xBL;
   long mask = (1L << 48) - 1;
   seed = (seed * multiplier + addend) & (mask);
   uint result = seed >> (48 - bits);

   return (uint2) (result, seed);
}

/**
 * Taking in a random number seed, returns the random number and the modified seed, and forces this result
 * to be within the provided upper bound.
 **/
uint2 nextRandomInt(uint seed, uint bound)
{
   uint bits = 31;
   uint2 result = nextRandom(seed, bits);
   uint r = result.s0;
   seed = result.s1;

   uint m = bound - 1;
   if ((bound & m) == 0)
      r = (uint) ((bound * (long) r) >> bits);
   else
   {
      uint u = r;
      r = u % bound;
      while (u - r + m < 0)
      {
         result = nextRandom(seed, bits);
         u = result.s0;
         seed = result.s1;
         r = u % bound;
      }
   }

   return result;
}

/**
 * Checks whether variable a is within the value of variable b by some epsilon. Returns true if it is.
 **/
bool epsilonEquals(float a, float b, float epsilon)
{
   return fabs(a - b) < epsilon;
}

/**
 * Returns the signed distane of a point 3D to a plane 3D that's defined by the point on the plane and the plane normal.
 * If the distance is positive, the point is "above" the plane, where above is defined as the positive Z of the plane normal.
 * If the distance is negative, the point is below the plane.
 **/
float signedDistanceFromPoint3DToPlane3D(float3 pointQuery, float3 pointOnPlane, float3 planeNormal)
{
   float3 delta = pointQuery - pointOnPlane;

   return dot(delta, planeNormal);
}

/**
 * Returns the distance of a point 3D to a plane 3D that's defined by the point on the plane and the plane normal.
 **/
float distanceFromPoint3DToPlane3D(float3 pointQuery, float3 pointOnPlane, float3 planeNormal)
{
   float3 delta = pointQuery - pointOnPlane;

   return fabs(dot(delta, planeNormal));
}

/**
 * Returns the distance of a point 3D to a plane 3D that's defined by the point on the plane and the plane normal.
 **/
float getZOnPlane(float2 pointQuery, float3 pointOnPlane, float3 planeNormal)
{
   float k = dot(planeNormal, pointOnPlane);

   float residual = k - planeNormal.x * pointOnPlane.x - planeNormal.y * pointOnPlane.y;

   return residual / planeNormal.z;
}

/**
 * Computes the 3D normal vector to the plane defined by three points.
 **/
float3 computeNormal3DFromThreePoint3Ds(float3 firstPointOnPlane, float3 secondPointOnPlane, float3 thirdPointOnPlane)
{
   float3 v1 = secondPointOnPlane - firstPointOnPlane;
   float3 v2 = thirdPointOnPlane - firstPointOnPlane;

   float3 normal = cross(v1, v2);

   return normalize(normal);
}

void solveForPlaneCoefficients(float* covariance_matrix, float* z_variance_vector, float* coefficients)
{
    float* inverse_covariance_matrix = invert3x3Matrix(covariance_matrix);

    // This is a simple matrix multiply, coefficients = inverse_covariance_matrix * z_variance_vector
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            coefficients[row] += inverse_covariance_matrix[row * 3 + col] * z_variance_vector[col];
        }
    }
}
