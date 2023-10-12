void kernel heatMapKernel(read_write image2d_t inputValueImage,
                           read_write image2d_t heatMap,
                           global float *params)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   write_imageui(heatMap, (int2)(xIndex, yIndex), (uint4)(255, 128, 64, 0));


}
