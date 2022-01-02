kernel void manipulateImage(read_write image2d_t image)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   write_imagef(image, (int2) (x, y), (float4) (x, 0.0, 0.0, 1.0));
}