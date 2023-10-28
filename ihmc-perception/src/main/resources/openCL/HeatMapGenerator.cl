void kernel heatMapKernel(read_write image2d_t inputValueImage,
                           read_write image2d_t heatMap,
                           global float *params)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   uint value = read_imageui(inputValueImage, (int2)(xIndex, yIndex)).x;

   write_imageui(heatMap, (int2)(xIndex, yIndex), (uint4)(0, 0, 255 - value, 255));


}
