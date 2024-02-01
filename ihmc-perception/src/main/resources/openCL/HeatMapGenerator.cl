void kernel heatMapKernel(read_write image2d_t inputValueImage,
                           read_write image2d_t heatMap,
                           global float *params)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   uint value = read_imageui(inputValueImage, (int2)(xIndex, yIndex)).x;

   uint green = (uint) clamp((uint)((value) * 30), (uint) 0, (uint) 255);
   uint red = 0;
   uint blue = 0;

   write_imageui(heatMap, (int2)(xIndex, yIndex), (uint4)(blue, green, red, 255));


}
