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

    printf("index: %d\n", index);
    printf("center: %f\n", *center);
    printf("resolution: %f\n", *resolution);
    printf("centerIndex: %d\n", centerIndex);

    int2 indexForConversion = make_int2(index, 1);
    float2 centerForConversion = make_float2(center[gid], center[gid]);
    float resolutionForConversion = resolution[gid];
    int centerIndexForConversion = centerIndex;

    float2 resultCoordinate = indices_to_coordinate(indexForConversion,
                                                    centerForConversion,
                                                    resolutionForConversion,
                                                    centerIndexForConversion);
    xResultCopiedToHost[gid] = resultCoordinate.x;
    printf("xResult on kernel: %f\n", *xResultCopiedToHost);
    yResultCopiedToHost[gid] = resultCoordinate.y;
    printf("yResult on kernel: %f\n", *yResultCopiedToHost);

}
