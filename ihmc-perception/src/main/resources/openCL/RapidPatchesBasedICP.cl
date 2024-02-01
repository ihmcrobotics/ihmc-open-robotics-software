/*
 * Rapid Planar Patches Based Iterative Closest Point Kernel
 */
void kernel icpKernel(global float* previous, global float* current, global float* params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   if (cIndex == 0 && rIndex == 0)
      printf("ICP Kernel: %d %d\n", cIndex, rIndex);

   float nx = previous[(rIndex * 10 + cIndex) * 6 + 0];
   float ny = previous[(rIndex * 10 + cIndex) * 6 + 1];
   float nz = previous[(rIndex * 10 + cIndex) * 6 + 2];

   float cx = previous[(rIndex * 10 + cIndex) * 6 + 3];
   float cy = previous[(rIndex * 10 + cIndex) * 6 + 4];
   float cz = previous[(rIndex * 10 + cIndex) * 6 + 5];

   // if(cIndex == 0 && rIndex == 0)
   printf("ICP Kernel: %d %d, Value: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", cIndex, rIndex, nx, ny, nz, cx, cy, cz);
}
