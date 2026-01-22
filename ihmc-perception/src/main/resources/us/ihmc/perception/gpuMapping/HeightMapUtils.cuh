__device__ float index_to_coordinate(int index, float center, float resolution, int center_index)
{
    return (index - center_index) * resolution + center;
}

__device__ float2 indices_to_coordinate(int2 index, float2 center, float resolution, int center_index)
{
    return make_float2(index_to_coordinate(index.x, center.x, resolution, center_index), index_to_coordinate(index.y, center.y, resolution, center_index));
}

__device__ int coordinate_to_index(float coordinate, float center, float resolution, int center_index)
{
    return round((coordinate - center) / resolution) + center_index;
}

__device__ int2 coordinate_to_indices(float2 coordinates, float2 center, float resolution, int center_index)
{
    return make_int2(coordinate_to_index(coordinates.x, center.x, resolution, center_index), coordinate_to_index(coordinates.y, center.y, resolution, center_index));
}

__device__ int compute_center_index(float gridSize, float resolution)
{
    return round(0.5f * gridSize / resolution);
}