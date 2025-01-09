#include "HeightMapUtils.cuh"

extern "C"

__global__ void test_indices_to_coordinate(int index,
                                           float* center,
                                           float* resolution,
                                           int centerIndex,
                                           float* xResultCopiedToHost,
                                           float* yResultCopiedToHost)
{
    int gid = blockIdx.x * blockDim.x + threadIdx.x;

    printf("Index: %d\n", index);
    printf("Center: %f\n", *center);
    printf("Resolution: %f\n", *resolution);
    printf("CenterIndex: %d\n", centerIndex);

    int2 indexForConversion = make_int2(index, 1);
    float2 centerForConversion = make_float2(center[gid], center[gid]);
    float resolutionForConversion = resolution[gid];
    int centerIndexForConversion = centerIndex;

    float2 resultCoordinate = indices_to_coordinate(indexForConversion,
                                                    centerForConversion,
                                                    resolutionForConversion,
                                                    centerIndexForConversion);
    xResultCopiedToHost[gid] = resultCoordinate.x;
    printf("X Result on kernel: %f\n", *xResultCopiedToHost);
    yResultCopiedToHost[gid] = resultCoordinate.y;
    printf("Y Result on kernel: %f\n", *yResultCopiedToHost);
}

extern "C"
__global__ void test_coordinate_to_indices(float* coordinate,
                                           float* center,
                                           float resolution,
                                           int center_index,
                                           float* xResultCopiedToHost,
                                           float* yResultCopiedToHost)
{
    int gid = blockIdx.x * blockDim.x + threadIdx.x;

    printf("Coordinates: (%f, %f)\n", *coordinate, *coordinate);
    printf("Center: (%f, %f)\n", *center, *center);
    printf("Resolution: %f\n", resolution);
    printf("CenterIndex: %d\n", center_index);

    float2 coordinateForConversion = make_float2(*coordinate, *coordinate);
    float2 centerForConversion = make_float2(center[gid], center[gid]);
    float resolutionForConversion = resolution;
    int centerIndexForConversion = center_index;

    int2 resultIndices = coordinate_to_indices(coordinateForConversion,
                                               centerForConversion,
                                               resolutionForConversion,
                                               centerIndexForConversion);

    xResultCopiedToHost[gid] = resultIndices.x;
    printf("X Result on kernel: %f\n", *xResultCopiedToHost);
    yResultCopiedToHost[gid] = resultIndices.y;
    printf("Y Result on kernel: %f\n", *yResultCopiedToHost);
}