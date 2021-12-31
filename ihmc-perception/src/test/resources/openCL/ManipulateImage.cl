kernel void manipulateImage(read_write image2d_t image)
{
   int y = get_global_id(0);
   int x = get_global_id(1);
   int2 coordinates = (int2) (x,y);

   // make the pixel black
   write_imageui(image, coordinates, 2.5);
}