extern "C"

__device__ float index_to_coordinate(int index, float center, float resolution, int center_index)
{
    return (index - center_index) * resolution + center;
}

__device__ float2 indices_to_coordinate(int2 index, float2 center, float resolution, int center_index)
{
    return make_float2(index_to_coordinate(index.x, center.x, resolution, center_index), index_to_coordinate(index.y, center.y, resolution, center_index));
}