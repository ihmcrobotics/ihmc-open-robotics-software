#include "Utils.cu"
#include "MathUtils.cuh"
#include "PerceptionUtils.cu"

using namespace PerceptionUtils;

extern "C"
#define IMAGE_HEIGHT 0
#define IMAGE_WIDTH 1
#define PATCH_IMAGE_HEIGHT 2
#define PATCH_IMAGE_WIDTH 3
#define PATCH_SIZE 4
#define DEPTH_FX 5
#define DEPTH_FY 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define NORMAL_PACK_RANGE 9
#define MERGE_RANGE 10
#define MERGE_ANGULAR_THRESHOLD 11
#define MERGE_ORTHOGONAL_THRESHOLD 12
#define MERGE_DISTANCE_THRESHOLD 13

/**
 * From "zOut" where z points out of the image to "zUp" where z points along the image plane
 */
__device__ float3 toZUp(float3 zOut)
{
    return make_float3(zOut.z, -zOut.x, -zOut.y);
}

/**
 * From "zUp" where z points along the image plane to "zOut" where z points out of the image
 */
__device__ float3 toZOut(float3 zUp)
{
    return make_float3(-zUp.y, -zUp.z, zUp.x);
}

/**
 * Back projects the depth image at pixel (u, v) given the depth image in mm
 */
__device__ float3 backProject(int u, int v, const unsigned short* __restrict__ depthImage, long pitchDepthImage, const float* __restrict__ params)
{
    // Depth image is in mm
    unsigned short depthMM = get2d(depthImage, pitchDepthImage, u, v);

    // Scale to meters
    float depth = 0.001f * depthMM;

    // Pinhole back projection
    float x = depth * (u - params[DEPTH_CX]) / params[DEPTH_FX];
    float y = depth * (v - params[DEPTH_CY]) / params[DEPTH_FY];

    // Construct point in Z-out, i.e. depth points along z
    return make_float3(x, y, depth);
}

/**
* Computes whether two points are connected, i.e. on the same plane.
*/
__device__ bool isConnected(float3 pointA, float3 normalA, float3 pointB, float3 normalB, const float* __restrict__ params)
{
   float lenA = length(pointA);
   float lenB = length(pointB);

   float3 pMin = (lenB < lenA) ? pointB : pointA;
   float3 pMax = (lenB < lenA) ? pointA : pointB;
   float3 connectingVector = pMax - pMin;

   float distanceToNearPoint = min(lenA, lenB);
   float distanceToFarPoint = max(lenA, lenB);

   float distanceAtoB = length(connectingVector);
   float normalDotProduct = fabs(dot(normalA, normalB));

   float perpDist = max(fabs(dot(pointA - pointB, normalB)), fabs(dot(pointB - pointA, normalA)));

   // Camera (O)
   //    /\
   //   /  \
   //  A----B
   // theta = angle at O
   // phi = angle at nearest point
   float theta = acos(dot(pointA, pointB) / (length(pointA) * length(pointB)));
   float phi = acos(fabs(dot(pMin, connectingVector) / (length(pMin) * length(connectingVector))));

   // Apply law of sines to compute predicted distance between points
   float distThreshold = distanceToFarPoint * fabs(sin(theta) / sin(phi));

   bool isCloseOrthogonal = perpDist < params[MERGE_ORTHOGONAL_THRESHOLD]; // both points lie near the plane defined by the neighbor
   bool isClosePerspective = distanceAtoB < distThreshold * params[MERGE_DISTANCE_THRESHOLD];  // ensures the separation is geometrically plausible under perspective
   bool isCloseAngular = normalDotProduct > params[MERGE_ANGULAR_THRESHOLD]; // both points have similar normals
   bool isNotTooCloseToCamera = distanceToNearPoint > 0.5f; // nearest point is not too close to the camera

   // TODO remove or increase this threshold before merging
   bool isNotTooFarFromCamera = distanceToFarPoint < 1.6f; // nearest point is not too far away (do not consider planar regions very far away)

   return isCloseOrthogonal && isClosePerspective && isCloseAngular && isNotTooCloseToCamera && isNotTooFarFromCamera;
}

/**
 * Performs normal estimation by sampling sets of points and computing cross-products.
 * See III-B in "GPU-Accelerated Rapid Planar Region...", doi: 10.1109/IROS51168.2021.9636009
 */
__device__ float3 estimateNormalPCA(const unsigned short* __restrict__ depthImage, long pitchDepthImage, const float* __restrict__ params, size_t patchXIndex, size_t patchYIndex)
{
    int patchSize = (int) params[PATCH_SIZE];
    int normalPackRange = clamp((int) params[NORMAL_PACK_RANGE], 1, patchSize - 1);
    int count = 0;
    float3 normal = make_float3(0.0f, 0.0f, 0.0f);

    if (patchYIndex < (int) params[PATCH_IMAGE_HEIGHT] && patchXIndex < (int) params[PATCH_IMAGE_WIDTH])
    {
        for (int i = 0; i < (int) params[PATCH_SIZE] - normalPackRange; i++)
        {
            for (int j = 0; j < (int) params[PATCH_SIZE] - normalPackRange; j++)
            {
                count++;
                int uQuery = patchXIndex * (int) params[PATCH_SIZE] + i;
                int vQuery = patchYIndex * (int) params[PATCH_SIZE] + j;

                float3 pointA = backProject(uQuery, vQuery, depthImage, pitchDepthImage, params);
                float3 pointB = backProject(uQuery + normalPackRange, vQuery, depthImage, pitchDepthImage, params);
                float3 pointC = backProject(uQuery + normalPackRange, vQuery + normalPackRange, depthImage, pitchDepthImage, params);
                float3 pointD = backProject(uQuery, vQuery + normalPackRange, depthImage, pitchDepthImage, params);

                normal = normal + cross3((pointC - pointB), (pointB - pointA));
                normal = normal + cross3((pointD - pointC), (pointC - pointB));
                normal = normal + cross3((pointA - pointD), (pointD - pointC));
                normal = normal + cross3((pointB - pointA), (pointA - pointD));
            }
        }
    }

    return normalize(normal);
}

/*
* Compute and pack centroids and normal of the patch
*/
extern "C"
__global__ void packKernel(const unsigned short* __restrict__ depthImage,
                           float* __restrict__ normalXMap, float* __restrict__ normalYMap, float* __restrict__ normalZMap,
                           float* __restrict__ centroidXMap, float* __restrict__ centroidYMap, float* __restrict__ centroidZMap,
                           long pitchDepthImage, long pitchFeatureMaps,
                           const float* __restrict__ params)
{
    int patchXIndex = Utils::getThreadCoordX();
    int patchYIndex = Utils::getThreadCoordY();

    int patchImageWidth = params[PATCH_IMAGE_WIDTH];
    int patchImageHeight = params[PATCH_IMAGE_HEIGHT];

    if (patchXIndex >= patchImageWidth || patchYIndex >= patchImageHeight)
        return;

    int imageWidth = params[IMAGE_WIDTH];
    int imageHeight = params[IMAGE_HEIGHT];
    int patchSize = params[PATCH_SIZE];

    // Compute centroid over patch
    int minU = max(patchXIndex * patchSize, 0);
    int maxU = min((patchXIndex + 1) * patchSize - 1, imageWidth - 1);
    int minV = max(patchYIndex * patchSize, 0);
    int maxV = min((patchYIndex + 1) * patchSize - 1, imageHeight - 1);

    int uSize = maxU - minU + 1;
    int vSize = maxV - minV + 1;
    int numberOfPoints = uSize * vSize;

    float3 centroid = make_float3(0.0f, 0.0f, 0.0f);
    for (int du = 0; du < uSize; du++)
    {
        for (int dv = 0; dv < vSize; dv++)
        {
            int uQuery = minU + du;
            int vQuery = minV + dv;
            float3 point = backProject(uQuery, vQuery, depthImage, pitchDepthImage, params);
            centroid = centroid + point;
        }
    }
    centroid = centroid / numberOfPoints;

    // Options 1, 2, 3 - compute covariance data
    CovarianceData covarianceData = {};

    for (int du = 0; du < uSize; du++)
    {
        for (int dv = 0; dv < vSize; dv++)
        {
            int uQuery = minU + du;
            int vQuery = minV + dv;
            float3 point = backProject(uQuery, vQuery, depthImage, pitchDepthImage, params);

            double dx = point.x - centroid.x;
            double dy = point.y - centroid.y;
            double dz = point.z - centroid.z;
            covarianceData.registerPoint(dx, dy, dz);
        }
    }

    covarianceData.numberOfPoints = (float) numberOfPoints;

    float coefficients[3] = {0.0f, 0.0f, 0.0f};
    float3 normal = make_float3(0.0f, 0.0f, 1.0f);

    // Option 1 -- compute through inverting 3x3 covariance matrix (determinants)
//     solveForPlaneCoefficients3x3_Determinants(covarianceData, coefficients);
//     normal.x = (float) coefficients[0];
//     normal.y = (float) coefficients[1];
//     normal = normalize(normal);

    // Option 2 -- compute through inverting 3x3 covariance matrix (Cholesky)
    solveForPlaneCoefficients3x3_Cholesky(covarianceData, coefficients);
    normal.x = (float) coefficients[0];
    normal.y = (float) coefficients[1];
    normal = normalize(normal);

    // Option 3 -- compute through inverting 2x2 covariance matrix
//     solveForPlaneNormal2x2(covarianceData, normal);
//     normal = normalize(normal);

    // Option 4 -- compute through principal component analysis, by area-weighted triangles
//     float3 normal = estimateNormalPCA(depthImage, pitchDepthImage, params, patchXIndex, patchYIndex);

    // Convert to Z-up
    centroid = toZUp(centroid);
    normal = toZUp(normal);

    // Pack patch feature data
    set2d(normalXMap, pitchFeatureMaps, patchXIndex, patchYIndex, normal.x);
    set2d(normalYMap, pitchFeatureMaps, patchXIndex, patchYIndex, normal.y);
    set2d(normalZMap, pitchFeatureMaps, patchXIndex, patchYIndex, normal.z);

    set2d(centroidXMap, pitchFeatureMaps, patchXIndex, patchYIndex, centroid.x);
    set2d(centroidYMap, pitchFeatureMaps, patchXIndex, patchYIndex, centroid.y);
    set2d(centroidZMap, pitchFeatureMaps, patchXIndex, patchYIndex, centroid.z);
}

/*
* Merge: populate connectivity of neighboring patches
*/
extern "C"
__global__ void mergeKernel(float* __restrict__ normalXMap, float* __restrict__ normalYMap, float* __restrict__ normalZMap,
                           float* __restrict__ centroidXMap, float* __restrict__ centroidYMap, float* __restrict__ centroidZMap,
                           unsigned short* __restrict__ connectionsMap,
                           long pitchFeatureMaps, long pitchConnectionsMap,
                           const float* __restrict__ params)
{
    int patchXIndex = Utils::getThreadCoordX();
    int patchYIndex = Utils::getThreadCoordY();

    int mergeRange = params[MERGE_RANGE];
    int patchImageWidth = params[PATCH_IMAGE_WIDTH];
    int patchImageHeight = params[PATCH_IMAGE_HEIGHT];

    if (patchXIndex < mergeRange || patchYIndex < mergeRange || patchXIndex >= patchImageWidth - mergeRange || patchYIndex >= patchImageHeight - mergeRange)
        return;

    float centroidX = get2d(centroidXMap, pitchFeatureMaps, patchXIndex, patchYIndex);
    float centroidY = get2d(centroidYMap, pitchFeatureMaps, patchXIndex, patchYIndex);
    float centroidZ = get2d(centroidZMap, pitchFeatureMaps, patchXIndex, patchYIndex);
    float3 centroid = make_float3(centroidX, centroidY, centroidZ);

    float normalX = get2d(normalXMap, pitchFeatureMaps, patchXIndex, patchYIndex);
    float normalY = get2d(normalYMap, pitchFeatureMaps, patchXIndex, patchYIndex);
    float normalZ = get2d(normalZMap, pitchFeatureMaps, patchXIndex, patchYIndex);
    float3 normal = make_float3(normalX, normalY, normalZ);

    unsigned short boundaryConnectionsEncodedAsOnes = 0;
    int count = 0;

    for (int signX = -1; signX <= 1; signX++)
    {
        for (int signY = -1; signY <= 1; signY++)
        {
            if (signX == 0 && signY == 0)
                continue;

            int xNeighbor = patchXIndex + signX * mergeRange;
            int yNeighbor = patchYIndex + signY * mergeRange;

            float centroidNeighborX = get2d(centroidXMap, pitchFeatureMaps, xNeighbor, yNeighbor);
            float centroidNeighborY = get2d(centroidYMap, pitchFeatureMaps, xNeighbor, yNeighbor);
            float centroidNeighborZ = get2d(centroidZMap, pitchFeatureMaps, xNeighbor, yNeighbor);
            float3 centroidNeighbor = make_float3(centroidNeighborX, centroidNeighborY, centroidNeighborZ);

            float normalNeighborX = get2d(normalXMap, pitchFeatureMaps, xNeighbor, yNeighbor);
            float normalNeighborY = get2d(normalYMap, pitchFeatureMaps, xNeighbor, yNeighbor);
            float normalNeighborZ = get2d(normalZMap, pitchFeatureMaps, xNeighbor, yNeighbor);
            float3 normalNeighbor = make_float3(normalNeighborX, normalNeighborY, normalNeighborZ);

            if (isConnected(centroid, normal, centroidNeighbor, normalNeighbor, params))
            {
                boundaryConnectionsEncodedAsOnes = (1 << count) | boundaryConnectionsEncodedAsOnes;
            }
            count++;
        }
    }

    set2d(connectionsMap, pitchConnectionsMap, patchXIndex, patchYIndex, boundaryConnectionsEncodedAsOnes);
}