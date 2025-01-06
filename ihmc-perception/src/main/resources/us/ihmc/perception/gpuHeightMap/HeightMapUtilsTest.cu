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