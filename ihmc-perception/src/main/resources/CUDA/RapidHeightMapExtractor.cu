

__global__ void heightMapUpdateKernel(
    cudaTextureObject_t in,
    cudaTextureObject_t out,
    float *params,
    float *sensorToZUpFrameTf,
    float *zUpToSensorFrameTf)
{
    // Get the thread's unique x and y indices
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Ensure the indices are within the bounds of the image
    if (xIndex >= (int)params[DEPTH_INPUT_WIDTH] || yIndex >= (int)params[DEPTH_INPUT_HEIGHT])
        return;

    float currentAverageHeight = 0.0f;
    float averageHeightZ = 0.0f;
    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, 0.5f);
    cellCenterInZUp.xy = indices_to_coordinate(make_int2(xIndex, yIndex),
                                               make_float2(0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[LOCAL_CELL_SIZE],
                                               params[LOCAL_CENTER_INDEX]);

    cellCenterInZUp.x += params[GRID_OFFSET_X];

    float halfCellWidth = params[LOCAL_CELL_SIZE] / 2.0f;
    float minX = cellCenterInZUp.x - halfCellWidth;
    float maxX = cellCenterInZUp.x + halfCellWidth;
    float minY = cellCenterInZUp.y - halfCellWidth;
    float maxY = cellCenterInZUp.y + halfCellWidth;

    int count = 0;
    int skip = (int)params[SEARCH_SKIP_SIZE];

    float3 cellCenterInSensor = transformPoint3D32_2(
        cellCenterInZUp,
        make_float3(sensorToZUpFrameTf[0], sensorToZUpFrameTf[1], sensorToZUpFrameTf[2]),
        make_float3(sensorToZUpFrameTf[4], sensorToZUpFrameTf[5], sensorToZUpFrameTf[6]),
        make_float3(sensorToZUpFrameTf[8], sensorToZUpFrameTf[9], sensorToZUpFrameTf[10]),
        make_float3(sensorToZUpFrameTf[3], sensorToZUpFrameTf[7], sensorToZUpFrameTf[11]));

    int2 projectedPoint;
    if (params[MODE] == 0) // Spherical Projection
    {
        projectedPoint = spherical_projection(cellCenterInSensor, params);
    }
    else if (params[MODE] == 1) // Perspective Projection
    {
        // Convert cellCenterInSensor to z-forward, x-right, y-down
        float3 cellCenterInSensorZfwd = make_float3(-cellCenterInSensor.y, -cellCenterInSensor.z, cellCenterInSensor.x);

        if (cellCenterInSensorZfwd.z < 0)
            return;

        projectedPoint = perspective_projection(cellCenterInSensorZfwd, params);
    }

    for (int pitch_count_offset = -((int)params[SEARCH_WINDOW_HEIGHT] / 2);
         pitch_count_offset < ((int)params[SEARCH_WINDOW_HEIGHT] / 2 + 1);
         pitch_count_offset += skip)
    {
        int pitch_count = projectedPoint.y + pitch_count_offset;
        for (int yaw_count_offset = -((int)params[SEARCH_WINDOW_WIDTH] / 2);
             yaw_count_offset < ((int)params[SEARCH_WINDOW_WIDTH] / 2 + 1);
             yaw_count_offset += skip)
        {
            int yaw_count = projectedPoint.x + yaw_count_offset;
            if ((yaw_count >= 0) && (yaw_count < (int)params[DEPTH_INPUT_WIDTH]) &&
                (pitch_count >= 0) && (pitch_count < (int)params[DEPTH_INPUT_HEIGHT]))
            {
                float depth = tex2D<float>(in, yaw_count, pitch_count) / 1000.0f;
                float3 queryPointInSensor;
                if (params[MODE] == 0) // Spherical
                {
                    queryPointInSensor = back_project_spherical(yaw_count, pitch_count, depth, params);
                }
                else if (params[MODE] == 1) // Perspective
                {
                    queryPointInSensor = back_project_perspective(make_int2(yaw_count, pitch_count), depth, params);
                }

                float3 queryPointInZUp = transformPoint3D32_2(
                    queryPointInSensor,
                    make_float3(sensorToZUpFrameTf[0], sensorToZUpFrameTf[1], sensorToZUpFrameTf[2]),
                    make_float3(sensorToZUpFrameTf[4], sensorToZUpFrameTf[5], sensorToZUpFrameTf[6]),
                    make_float3(sensorToZUpFrameTf[8], sensorToZUpFrameTf[9], sensorToZUpFrameTf[10]),
                    make_float3(sensorToZUpFrameTf[3], sensorToZUpFrameTf[7], sensorToZUpFrameTf[11]));

                if (queryPointInZUp.x > minX && queryPointInZUp.x < maxX &&
                    queryPointInZUp.y > minY && queryPointInZUp.y < maxY)
                {
                    // Remove outliers before averaging for a single cell
                    if (count > 1)
                    {
                        currentAverageHeight = averageHeightZ / (float)(count);
                        if (fabs(queryPointInZUp.z - currentAverageHeight) > 0.1f)
                        {
                            continue;
                        }
                    }
                    count++;
                    averageHeightZ += queryPointInZUp.z;
                }
            }
        }
    }

    if (count > 0)
    {
        averageHeightZ = averageHeightZ / (float)(count);
    }
    else
    {
        // This is slightly below the floor height of what we'll accept
        averageHeightZ = -params[HEIGHT_OFFSET];
    }
    averageHeightZ = clamp(averageHeightZ, params[MIN_CLAMP_HEIGHT], params[MAX_CLAMP_HEIGHT]);
    averageHeightZ += params[HEIGHT_OFFSET];

    // Write the result to the output texture
    uint4 outputValue = make_uint4(static_cast<int>((averageHeightZ) * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0);
    surf2Dwrite(outputValue, out, yIndex * sizeof(uint4), xIndex);
}