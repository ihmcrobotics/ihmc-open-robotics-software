int key_to_x_index(int key, int center_index)
{
   return key % (2 * center_index + 1);
}

int key_to_y_index(int key, int center_index)
{
   return key / (2 * center_index + 1);
}

int indices_to_key(int x_index, int y_index, int centerIndex)
{
   return x_index + y_index * (2 * centerIndex + 1);
}

int coordinate_to_index(float coordinate, float center, float resolution, int center_index)
{
   return round((coordinate - center) / resolution) + center_index;
}

int coordinate_to_key(float2 coordinates, float2 center, float resolution, int center_index)
{
   int x_index = coordinate_to_index(coordinates.x, center.x, resolution, center_index);
   int y_index = coordinate_to_index(coordinates.y, center.y, resolution, center_index);
   return indices_to_key(x_index, y_index, center_index);
}

float index_to_coordinate(int index, float center, float resolution, int center_index)
{
   return (index - center_index) * resolution + center;
}

float2 indices_to_coordinate(int2 index, float2 center, float resolution, int center_index)
{
   return (float2) (index_to_coordinate(index.x, center.x, resolution, center_index), index_to_coordinate(index.y, center.y, resolution, center_index));
}

float2 key_to_coordinate(int key, float2 center, float resolution, int center_index)
{
   int2 index = (int2) (key_to_x_index(key, center_index), key_to_y_index(key, center_index));
   return indices_to_coordinate(index, center, resolution, center_index);
}

int2 coordinate_to_indices(float2 coordinates, float2 center, float resolution, int center_index)
{
   return (int2) (coordinate_to_index(coordinates.x, center.x, resolution, center_index), coordinate_to_index(coordinates.y, center.y, resolution, center_index));
}