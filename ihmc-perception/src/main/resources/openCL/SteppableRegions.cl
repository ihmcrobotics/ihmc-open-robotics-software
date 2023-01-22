#define HEIGHT_MAP_CENTER_INDEX 0
#define HEIGHT_MAP_RESOLUTION 1
#define MIN_DISTANCE_FROM_CLIFF_TOPS 2
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 3
#define YAW_DISCRETIZATIONS 4
#define FOOT_WIDTH 5
#define FOOT_LENGTH 6

void kernel computeSteppability(global float* params,
                                read_only image2d_t height_map)
{
    // Remember, these are x and y in image coordinates, not world
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);
    int idx_yaw = get_global_id(2);

    int2 key = (int2) (idx_x, idx_y);
    float Z = (float) read_imagef(height_map, key).x;
}