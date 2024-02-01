#define r00 0
#define r01 1
#define r02 2
#define r10 3
#define r11 4
#define r12 5
#define r20 6
#define r21 7
#define r22 8
#define tx 9
#define ty 10
#define tz 11

float transform_p(float x, float y, float z, float r0, float r1, float r2, float t)
{
   return r0 * x + r1 * y + r2 * z + t;
}

void kernel transformPointsKernel(global float* points_in, global float* localization, global float* points_out)
{
   int i = get_global_id(0);

   float rx = points_in[i * 3];
   float ry = points_in[i * 3 + 1];
   float rz = points_in[i * 3 + 2];

   float x = transform_p(rx, ry, rz, localization[r00], localization[r01], localization[r02], localization[tx]);
   float y = transform_p(rx, ry, rz, localization[r10], localization[r11], localization[r12], localization[ty]);
   float z = transform_p(rx, ry, rz, localization[r20], localization[r21], localization[r22], localization[tz]);

   points_out[i * 3] = x;
   points_out[i * 3 + 1] = y;
   points_out[i * 3 + 2] = z;
}
