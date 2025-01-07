void kernel test_math_utils_transform_point(global float* px,
                                            global float* py,
                                            global float* pz,
                                            global float* r0x,
                                            global float* r0y,
                                            global float* r0z,
                                            global float* r1x,
                                            global float* r1y,
                                            global float* r1z,
                                            global float* r2x,
                                            global float* r2y,
                                            global float* r2z,
                                            global float* tx,
                                            global float* ty,
                                            global float* tz,
                                            global float* result)
{
    int gid = get_global_id(0);

    // Load point coordinates and rotation matrix rows from buffers
    float3 point = (float3)(px[gid], py[gid], pz[gid]);
    float3 row0 = (float3)(r0x[gid], r0y[gid], r0z[gid]);
    float3 row1 = (float3)(r1x[gid], r1y[gid], r1z[gid]);
    float3 row2 = (float3)(r2x[gid], r2y[gid], r2z[gid]);
    float3 translation = (float3)(tx[gid], ty[gid], tz[gid]);

    // Call the transform function to get the transformed point
    float3 transformedPoint = transformPoint3D32_2(point, row0, row1, row2, translation);

    // Store the result in the output buffer
    result[gid * 3 + 0] = transformedPoint.x;
    result[gid * 3 + 1] = transformedPoint.y;
    result[gid * 3 + 2] = transformedPoint.z;

    // Optionally print for debugging
    printf("Point: (%f, %f, %f)\n", point.x, point.y, point.z);
    printf("Rotation Matrix Rows:\n");
    printf("Row 0: (%f, %f, %f)\n", row0.x, row0.y, row0.z);
    printf("Row 1: (%f, %f, %f)\n", row1.x, row1.y, row1.z);
    printf("Row 2: (%f, %f, %f)\n", row2.x, row2.y, row2.z);
    printf("Translation: (%f, %f, %f)\n", translation.x, translation.y, translation.z);
    printf("Transformed Point: (%f, %f, %f)\n", transformedPoint.x, transformedPoint.y, transformedPoint.z);
}