 void kernel addPointsKernel(read_only image2d_t in, write_only image2d_t filteredDepth, write_only image2d_t buffer_nx, global float* params)
 {
    int y = get_global_id(0);
    int x = get_global_id(1);

//    if(x==0 && y==0) printf("FilterKernel\n");
    if (y >= 0 && y < (int)params[FILTER_SUB_H] && x >= 0 && x < (int)params[FILTER_SUB_W])
    {
        fill_dead_pixels(in, x, y, filteredDepth, params);
//        mark_boundary_patches(in, x, y, buffer_nx, params);
//        smooth_non_boundary(in, x, y, filteredDepth, params);
    }
}
