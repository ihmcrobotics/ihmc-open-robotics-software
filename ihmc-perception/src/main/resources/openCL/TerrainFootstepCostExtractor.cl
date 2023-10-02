float compute_footstep_cost(global float* )
{
}

void kernel footstepCostKernel(read_write image2d_t heightMap,
                                       read_write image2d_t costMap,
                                       global float *params)
{
   // Extract the indices
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   // Compute the cell center in the world frame
   float3 cellCenterInWorld = (float3)(0, 0, 0);
   cellCenterInWorld.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[GLOBAL_CELL_SIZE],
                                               params[GLOBAL_CENTER_INDEX]);

   // In a 5x5 neighborhood around the current cell compute the steppability as 4 booleans to be stored in the cost map first four bits
   int steppability = 0;
   int m = 2;
   for (int xOffset = -m; xOffset < m+1; xOffset++)
   {
      for (int yOffset = -m; yOffset < m+1; yOffset++)
      {
            compute_footstep_cost()
      }
   }


   // Put the steppability count for each out of 4 layers around the current cell as boolean in first 4 bits of the cost map



}