void kernel test_indices_to_coordinate(global int* index,
                                       global float* center,
                                       global float* resolution,
                                       global int* centerIndex,
                                       global float* xResultCopiedToHost,
                                       global float* yResultCopiedToHost)
{
    int gid = get_global_id(0);

    printf("index: %d\n", index[gid]);
    printf("center: %f\n", center[gid]);
    printf("resolution: %f\n", resolution[gid]);
    printf("centerIndex: %d\n", centerIndex[gid]);

    int2 indexForConversion = (int2) (index[gid], 1);
    float2 centerForConversion = (float2) (center[gid], center[gid]);
    float resolutionForConversion = resolution[gid];
    int centerIndexForConversion = centerIndex[gid];

    float2 resultCoordinate = indices_to_coordinate(indexForConversion,
                                                    centerForConversion,
                                                    resolutionForConversion,
                                                    centerIndexForConversion);
    xResultCopiedToHost[gid] = resultCoordinate.x;
    yResultCopiedToHost[gid] = resultCoordinate.y;
}

void kernel test_coordinate_to_indices(global float* coordinate,
                                       global float* center,
                                       global float* resolution,
                                       global int* center_index,
                                       global float* xResultCopiedToHost,
                                       global float* yResultCopiedToHost)
{
    int gid = get_global_id(0);

    printf("index: %f\n", coordinate[gid]);
    printf("center: %f\n", center[gid]);
    printf("resolution: %f\n", resolution[gid]);
    printf("centerIndex: %d\n", center_index[gid]);

    float2 coordinateForConversion = (float2) (coordinate[gid], coordinate[gid]);
    float2 centerForConversion = (float2) (center[gid], center[gid]);
    float resolutionForConversion = resolution[gid];
    int centerIndexForConversion = center_index[gid];

    int2 resultCoordinate = coordinate_to_indices(coordinateForConversion,
                                                    centerForConversion,
                                                    resolutionForConversion,
                                                    centerIndexForConversion);

    xResultCopiedToHost[gid] = resultCoordinate.x;
    yResultCopiedToHost[gid] = resultCoordinate.y;
}

                                       