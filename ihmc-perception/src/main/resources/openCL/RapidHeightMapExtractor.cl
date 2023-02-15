#define GRID_LENGTH_METERS 0
#define GRID_WIDTH_METERS 1
#define CELL_SIZE_XY_METERS 2
#define INPUT_HEIGHT 3
#define INPUT_WIDTH 4
#define GRID_LENGTH 5
#define GRID_WIDTH 6

float3 back_project_spherical(int2 pos, float depth, global float* params)
{
   float totalPitch = M_PI / 2;
   float totalYaw = 2 * M_PI;

   int x = pos.x;
   int y = pos.y;

   int xFromCenter = -x - (params[INPUT_WIDTH] / 2);
   int yFromCenter = -(y - (params[INPUT_HEIGHT] / 2));

   float yaw = xFromCenter / (float) params[INPUT_WIDTH] * totalYaw;
   float pitch = yFromCenter / (float) params[INPUT_HEIGHT] * totalPitch;

    float r = depth * cos(pitch);

    float px = r * cos(yaw);
    float py = r * sin(yaw);
    float pz = depth * sin(pitch);

    //printf("Projection: [%d,%d] (X:%.2lf,Y:%.2lf,Z:%.2lf,R:%.2lf,D:%.2lf)\n", x, y, px, py, pz, r, depth);

    float3 X = (float3) (px, py, pz);
    return X;
}

float3 compute_grid_cell_center(int rIndex, int cIndex, global float* params)
{
   float3 cellCenter = (float3)(0, 0, 0);
   cellCenter.x = ((params[GRID_LENGTH] / 2) - rIndex) * params[CELL_SIZE_XY_METERS];
   cellCenter.y = ((params[GRID_WIDTH] / 2) - cIndex) * params[CELL_SIZE_XY_METERS];
   cellCenter.z = -2.0f;
   return cellCenter;
}

int2 spherical_projection(float3 cellCenter, global float* params)
{
    int2 proj = (int2)(0, 0);

    int count = 0;
    float pitchUnit = M_PI / (2 * params[INPUT_HEIGHT]);
    float yawUnit = 2 * M_PI / (params[INPUT_WIDTH]);

    int pitchOffset = params[INPUT_HEIGHT]/2;
    int yawOffset = params[INPUT_WIDTH]/2;

    float x = cellCenter.x;
    float y = cellCenter.y;
    float z = cellCenter.z;

    float radius = length(cellCenter.xy);

    float pitch = atan2(z, radius);
    int pitchCount = (pitchOffset) - (int) (pitch / pitchUnit);

    float yaw = atan2(-y, x);
    int yawCount = (yawOffset) + (int) (yaw / yawUnit);

    proj.x = pitchCount;
    proj.y = yawCount;

//    printf("Projection: [%.2f,%.2f] (Yc:%d,Pc:%d, Z:%.2lf,R:%.2lf)\n", yaw, pitch, yawCount, pitchCount, z, radius);

    return proj;
}

float get_height_on_plane(float x, float y, global float* plane)
{
    float height = (plane[3] - (plane[0] * x + plane[1] * y)) / plane[2];
    return height;
}

void kernel heightMapUpdateKernel(  read_only image2d_t in, read_write image2d_t out, global float* params, global float* sensorToWorldTf,
                                    global float* worldToSensorTf, global float* plane)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

    // printf("Height Map Update Kernel\n");

    float3 normal;
    float3 centroid;

    float averageHeightZ = 0;
    float3 cellCenter = compute_grid_cell_center(rIndex, cIndex, params);

    cellCenter = transform3(cellCenter,
            (float3)(worldToSensorTf[0], worldToSensorTf[1], worldToSensorTf[2]),
            (float3)(worldToSensorTf[4], worldToSensorTf[5], worldToSensorTf[6]),
            (float3)(worldToSensorTf[8], worldToSensorTf[9], worldToSensorTf[10]),
            (float3)(worldToSensorTf[3], worldToSensorTf[7], worldToSensorTf[11]));

    int2 projectedPoint = spherical_projection(cellCenter, params);

    int2 pos = (int2)(0,0);

//    printf("[%d/%d, %d/%d] -> Cell: (%.2f, %.2f, %.2f) -> Proj: (%d,%d)\n", rIndex, (int) params[GRID_LENGTH],
//         cIndex, (int) params[GRID_WIDTH], cellCenter.x, cellCenter.y, cellCenter.z, pos.x, pos.y);

    int WINDOW_WIDTH = 20;

    int count = 0;

    for(int i = 0; i < (int) params[INPUT_HEIGHT]; i++)
    {
        for(int j = -WINDOW_WIDTH / 2; j < WINDOW_WIDTH / 2 + 1; j++)
        {
            pos = (int2)(projectedPoint.y + j, i);

//            printf("[%d/%d, %d/%d] -> Cell: (%.2f, %.2f, %.2f) -> Proj: (%d,%d)\n", rIndex, (int) params[GRID_LENGTH],
//                              cIndex, (int) params[GRID_WIDTH], cellCenter.x, cellCenter.y, cellCenter.z, projectedPoint.x, projectedPoint.y);

            if( (pos.x >= 0) && (pos.x < (int) params[INPUT_WIDTH]) && (pos.y >= 0) && (pos.y < (int) params[INPUT_HEIGHT]))
            {
                float radius = ((float)read_imageui(in, pos).x)/(float)1000;

                float3 point = back_project_spherical(pos, radius, params);
//                point = transform3(point,
//                        (float3)(sensorToWorldTf[0], sensorToWorldTf[1], sensorToWorldTf[2]),
//                        (float3)(sensorToWorldTf[4], sensorToWorldTf[5], sensorToWorldTf[6]),
//                        (float3)(sensorToWorldTf[8], sensorToWorldTf[9], sensorToWorldTf[10]),
//                        (float3)(sensorToWorldTf[3], sensorToWorldTf[7], sensorToWorldTf[11]));


//               printf("[%d,%d] : [%d/%d, %d/%d] -> Cell: (%.2f, %.2f, %.2f) -> Proj: (%d,%d) -> BackProj: (%.2lf,%.2lf,%.2lf) -> GridCell: (%.2f,%.2f)\n", i, j, rIndex, (int) params[GRID_LENGTH],
//                        cIndex, (int) params[GRID_WIDTH], cellCenter.x, cellCenter.y, cellCenter.z, projectedPoint.x, projectedPoint.y, point.x, point.y, point.z,
//                        (point.x / 0.1), point.y / 0.1);

                float minX = cellCenter.x - params[CELL_SIZE_XY_METERS] / 2;
                float maxX = cellCenter.x + params[CELL_SIZE_XY_METERS] / 2;
                float minY = cellCenter.y - params[CELL_SIZE_XY_METERS] / 2;
                float maxY = cellCenter.y + params[CELL_SIZE_XY_METERS] / 2;

//                printf("[%d,%d] : [%d/%d, %d/%d] -> Cell: (%.2f, %.2f, %.2f) -> Proj: (%d,%d) -> BackProj: (%.2lf,%.2lf,%.2lf) -> GridCell: (%.2f,%.2f) -> Bounds(X): (%.2f, %.2f),  -> Bounds(Y): (%.2f, %.2f)\n", i, j, rIndex, (int) params[GRID_LENGTH],
//                              cIndex, (int) params[GRID_WIDTH], cellCenter.x, cellCenter.y, cellCenter.z, projectedPoint.x, projectedPoint.y, point.x, point.y, point.z,
//                              (cIndex * params[CELL_SIZE_XY_METERS]), (rIndex * params[CELL_SIZE_XY_METERS]), minX, maxX, minY, maxY);

                if ( point.x > minX && point.x < maxX && point.y > minY && point.y < maxY)
                {
                    count++;
                    averageHeightZ += point.z;

//                     printf("[%d,%d] : [%d/%d, %d/%d] -> Cell: (%.2f, %.2f, %.2f) -> Proj: (%d,%d) -> BackProj: (%.2lf,%.2lf,%.2lf) -> GridCell: (%.2f,%.2f) -> Bounds(X): (%.2f, %.2f),  -> Bounds(Y): (%.2f, %.2f)\n", i, j, rIndex, (int) params[GRID_LENGTH],
//                              cIndex, (int) params[GRID_WIDTH], cellCenter.x, cellCenter.y, cellCenter.z, projectedPoint.x, projectedPoint.y, point.x, point.y, point.z,
//                              (cIndex * params[CELL_SIZE_XY_METERS]), (rIndex * params[CELL_SIZE_XY_METERS]), minX, maxX, minY, maxY);
//
//                    printf("_________________________________++++++++++++++++++++++++ FOUND +++++++++++++++++++++++++++++++____________________________________\n");


                }
            }
        }
    }

    if(count > 0)
    {
        averageHeightZ = max(-20.0f, min(1.5f, averageHeightZ / (float)(count) - get_height_on_plane(cellCenter.x, cellCenter.y, plane)));
        write_imageui(out, (int2)(cIndex,rIndex), (uint4)((int)( (2.0f + averageHeightZ) * 10000.0f), 0, 0, 0));
    }
    else
    {
        write_imageui(out, (int2)(cIndex,rIndex), (uint4)(0, 0, 0, 0));
    }

//    averageHeightZ = cellCenter.x * cellCenter.x + cellCenter.y * cellCenter.y;
//    write_imageui(out, (int2)(cIndex,rIndex), (uint4)((int)( (2.0f + averageHeightZ) * 10000.0f), 0, 0, 0));
}
