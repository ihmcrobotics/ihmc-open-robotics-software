#define GRID_LENGTH_METERS 0
#define GRID_WIDTH_METERS 1
#define CELL_SIZE_XY_METERS 2
#define INPUT_HEIGHT 3
#define INPUT_WIDTH 4

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

float4 transform4(float4 point, float4 r1, float4 r2, float4 r3, float4 t)
{
   return (float4)(dot(r1, point) + t.x, dot(r2, point) + t.y, dot(r3, point) + t.z, 0);
}

float3 compute_grid_cell_cell_center(int rIndex, int cIndex, global float* params)
{
   float3 cellCenter = (float3)(0, 0, 0);
   cellCenter.y = (cIndex - (params[GRID_WIDTH_METERS] / 2)) * params[CELL_SIZE_XY_METERS];
   cellCenter.x = (rIndex - (params[GRID_LENGTH_METERS] / 2)) * params[CELL_SIZE_XY_METERS];
   cellCenter.z = 0;
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
    int pitchCount = (pitchOffset) + (int) (pitch / pitchUnit);

    float yaw = atan2(-y, x);
    int yawCount = (yawOffset) + (int) (yaw / yawUnit);

    proj.x = pitchCount;
    proj.y = yawCount;

    return proj;
}

void kernel heightMapUpdateKernel(  read_write image2d_t in, write_only image2d_t out, global float* params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

    float3 normal;
    float3 centroid;

    float averageHeightZ = 0;
    float3 cellCenter = compute_grid_cell_cell_center(rIndex, cIndex, params);
    int2 projectedPoint = spherical_projection(cellCenter, params);

    int2 pos = (int2)(projectedPoint.x, projectedPoint.y);

    int WINDOW_LENGTH = 10;
    int WINDOW_WIDTH = 10;

    int count = 0;

    for(int i = 0; i<WINDOW_LENGTH; i++)
    {
        for(int j = 0; j<WINDOW_WIDTH; j++)
        {
            pos = (int2)(projectedPoint.x + i, projectedPoint.y + j);

            if(pos.x >= 0 && pos.x < params[INPUT_WIDTH] && pos.y >= 0 && pos.y < params[INPUT_HEIGHT])
            {
                float radius = ((float)read_imageui(in, pos).x)/(float)1000;

                float3 point = back_project_spherical(pos, radius, params);

                if ( (int) point.x == rIndex && (int) point.y == cIndex)
                {
                    count++;
                    averageHeightZ += point.z;
                }
            }
        }
    }

    averageHeightZ = averageHeightZ / (float)(count);

    write_imageui(out, (int2)(cIndex,rIndex), (uint4)(averageHeightZ * 10000, 0, 0, 0));

}
