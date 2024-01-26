kernel void segmentDepthImage(read_only image2d_t depthImage,
                              read_only image2d_t imageMask,
                              write_only image2d_t outputImage)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   uint maskValue = read_imageui(imageMask, (int2) (x, y)).x;
   float depthValue = read_imageui(depthImage, (int2) (x, y)).x;
   if (maskValue > 0)
   {
      write_imageui(outputImage, (int2) (x, y), depthValue);
   }
   else
      write_imageui(outputImage, (int2) (x, y), 0.0);
}