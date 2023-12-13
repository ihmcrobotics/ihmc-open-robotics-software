kernel void manipulateImage(write_only image2d_t image)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   if (y == 1)
      write_imagef(image, (int2) (x, y), (float4) (5.0, 0.0, 0.0, 1.0));
   else if (y > 2)
      write_imagef(image, (int2) (x, y), (float4) (x, 0.0, 0.0, 1.0));
}