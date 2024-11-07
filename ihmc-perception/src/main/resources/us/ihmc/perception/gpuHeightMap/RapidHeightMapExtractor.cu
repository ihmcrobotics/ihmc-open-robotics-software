
extern "C"
#define LOCAL_CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define HEIGHT_MAP_CENTER_X 4
#define HEIGHT_MAP_CENTER_Y 5
#define MODE 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define DEPTH_FX 9
#define DEPTH_FY 10
#define GLOBAL_CELL_SIZE 11
#define GLOBAL_CENTER_INDEX 12
#define ROBOT_COLLISION_RADIUS 13
#define GRID_OFFSET_X 14
#define HEIGHT_FILTER_ALPHA 15
#define LOCAL_CELLS_PER_AXIS 16
#define GLOBAL_CELLS_PER_AXIS 17
#define HEIGHT_SCALING_FACTOR 18
#define MIN_HEIGHT_REGISTRATION 19
#define MAX_HEIGHT_REGISTRATION 20
#define MIN_HEIGHT_DIFFERENCE 21
#define MAX_HEIGHT_DIFFERENCE 22
#define SEARCH_WINDOW_HEIGHT 23
#define SEARCH_WINDOW_WIDTH 24
#define CROPPED_WINDOW_CENTER_INDEX 25
#define MIN_CLAMP_HEIGHT 26
#define MAX_CLAMP_HEIGHT 27
#define HEIGHT_OFFSET 28
#define STEPPING_COSINE_THRESHOLD 29
#define STEPPING_CONTACT_THRESHOLD 30
#define CONTACT_WINDOW_SIZE 31
#define SPATIAL_ALPHA 32
#define SEARCH_SKIP_SIZE 33
#define VERTICAL_SEARCH_SIZE 34
#define VERTICAL_SEARCH_RESOLUTION 35
#define FAST_SEARCH_SIZE 36

#define VERTICAL_FOV 1.57079632679f
#define HORIZONTAL_FOV 6.28318530718f

#define SNAP_HEIGHT_MAP_CENTER_X 0
#define SNAP_HEIGHT_MAP_CENTER_Y 1
#define SNAP_GLOBAL_CELL_SIZE 2
#define SNAP_GLOBAL_CENTER_INDEX 3
#define SNAP_CROPPED_WINDOW_CENTER_INDEX 4
#define SNAP_HEIGHT_SCALING_FACTOR 5
#define SNAP_HEIGHT_OFFSET 6
#define SNAP_FOOT_LENGTH 7
#define SNAP_FOOT_WIDTH 8
#define MIN_DISTANCE_FROM_CLIFF_TOPS 9
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 10
#define CLIFF_START_HEIGHT_TO_AVOID 11
#define CLIFF_END_HEIGHT_TO_AVOID 12
#define MIN_SUPPORT_AREA_FRACTION 13
#define MIN_SNAP_HEIGHT_THRESHOLD 14
#define SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE 15
#define INEQUALITY_ACTIVATION_SLOPE 16

#define SNAP_FAILED 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define NOT_ENOUGH_AREA 0
#define VALID 4


__host__ __device__ float index_to_coordinate(int index, float center, float resolution, int center_index)
{
   return (index - center_index) * resolution + center;
}

__host__ __device__ float2 indices_to_coordinate(int2 index, float2 center, float resolution, int center_index)
{
return make_float2 (index_to_coordinate(index.x, center.x, resolution, center_index), index_to_coordinate(index.y, center.y, resolution, center_index));
}

__host__ __device__ float dot(const float3 &a, const float3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}



//
// __host__ __device__ int2 coordinate_to_indices(float2 coordinates, float2 center, float resolution, int center_index)
// {
//    return (int2) (coordinate_to_index(coordinates.x, center.x, resolution, center_index), coordinate_to_index(coordinates.y, center.y, resolution, center_index));
// }
__host__ __device__ int2 spherical_projection(float3 cellCenter, float *params)
{
float pitchUnit = VERTICAL_FOV / (params[DEPTH_INPUT_HEIGHT]);
float yawUnit = HORIZONTAL_FOV / (params[DEPTH_INPUT_WIDTH]);

int pitchOffset = params[DEPTH_INPUT_HEIGHT] / 2;
int yawOffset = params[DEPTH_INPUT_WIDTH] / 2;

float x = cellCenter.x;
float y = cellCenter.y;
float z = cellCenter.z;

float radius = sqrt(x * x + y * y);

float pitch = atan2f(z, radius);
int pitchCount = (pitchOffset) - static_cast<int>(pitch / pitchUnit);

float yaw = atan2f(-y, x);
int yawCount = (yawOffset) + static_cast<int>(yaw / yawUnit);

return make_int2 (pitchCount, yawCount);
}

__host__ __device__ int2 perspective_projection(float3 point, float* params)
{
float x = point.x / point.z * params[DEPTH_FX] + params[DEPTH_CX];
float y = point.y / point.z * params[DEPTH_FY] + params[DEPTH_CY];
return make_int2(static_cast<int>(x), static_cast<int>(y));
}
__host__ __device__ float3 back_project_spherical(int yaw_index, int pitch_index, float depth,  float *params)
{
int yawCountsFromCenter = -yaw_index - (params[DEPTH_INPUT_WIDTH] / 2);
int pitchCountsFromCenter = -(pitch_index - (params[DEPTH_INPUT_HEIGHT] / 2));

float yaw = yawCountsFromCenter / (float)params[DEPTH_INPUT_WIDTH] * HORIZONTAL_FOV;
float pitch = pitchCountsFromCenter / (float)params[DEPTH_INPUT_HEIGHT] * VERTICAL_FOV;

float r = depth * cos(pitch);

float px = r * cos(yaw);
float py = r * sin(yaw);
float pz = depth * sin(pitch);

return make_float3(px, py, pz);
}

__host__ __device__ float3 back_project_perspective(int2 pos, float Z, float* params)
{
float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
float3 point = make_float3 (Z, -X, -Y);
return point;
}

__host__ __device__ float3 transformPoint3D32_2(float3 point, float3 rotationMatrixRow0, float3 rotationMatrixRow1, float3 rotationMatrixRow2, float3 translation)
{
return make_float3 (dot(rotationMatrixRow0, point) + translation.x, dot(rotationMatrixRow1, point) + translation.y,
dot(rotationMatrixRow2, point) + translation.z);
}

__device__ float clamp(float value, float minVal, float maxVal)
{
    return fminf(fmaxf(value, minVal), maxVal);
}

__global__ void heightMapUpdateKernel( float3 *in,float3 *out,float *params,float *sensorToZUpFrameTf,float *zUpToSensorFrameTf)
{
int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

// if we need to calculate thread xIndex
float currentAverageHeight = 0.0f;
float averageHeightZ = 0.0f;
float3 cellCenterInZUp = make_float3(0.0f, 0.0f, 0.5f);

float2 xyCoords = indices_to_coordinate(make_int2(xIndex, yIndex), make_float2(0, 0), params[LOCAL_CELL_SIZE], params[LOCAL_CENTER_INDEX]);
cellCenterInZUp.x = xyCoords.x + params[GRID_OFFSET_X];
cellCenterInZUp.y = xyCoords.y;


float halfCellWidth = params[LOCAL_CELL_SIZE] / 2.0f;
float minX = cellCenterInZUp.x - halfCellWidth;
float maxX = cellCenterInZUp.x + halfCellWidth;
float minY = cellCenterInZUp.y - halfCellWidth;
float maxY = cellCenterInZUp.y + halfCellWidth;

int count = 0;
int skip = static_cast<int> (params[SEARCH_SKIP_SIZE]);

float3 cellCenterInSensor = transformPoint3D32_2(cellCenterInZUp,
            make_float3(zUpToSensorFrameTf[0], zUpToSensorFrameTf[1], zUpToSensorFrameTf[2]),
            make_float3(zUpToSensorFrameTf[4], zUpToSensorFrameTf[5], zUpToSensorFrameTf[6]),
            make_float3(zUpToSensorFrameTf[8], zUpToSensorFrameTf[9], zUpToSensorFrameTf[10]),
            make_float3(zUpToSensorFrameTf[3], zUpToSensorFrameTf[7], zUpToSensorFrameTf[11]));

  int2 projectedPoint;
   if (params[MODE] == 0) // Spherical Projection
   {
      projectedPoint = spherical_projection(cellCenterInSensor, params);
   }
   else if (params[MODE] == 1) // Perspective Projection
   {
      // convert cellCenterInSensor to z-forward, x-right, y-down
      float3 cellCenterInSensorZfwd = make_float3 (-cellCenterInSensor.y, -cellCenterInSensor.z, cellCenterInSensor.x);

      if (cellCenterInSensorZfwd.z < 0)
        return;

      projectedPoint = perspective_projection(cellCenterInSensorZfwd, params);
   }

for (int pitch_count_offset = -(int)(params[SEARCH_WINDOW_HEIGHT] / 2); pitch_count_offset < (int)(params[SEARCH_WINDOW_HEIGHT] / 2 + 1); pitch_count_offset += skip)
   {
      int pitch_count = projectedPoint.y + pitch_count_offset;
      for (int yaw_count_offset = - (int)( params[SEARCH_WINDOW_WIDTH] / 2); yaw_count_offset <  (int)( params[SEARCH_WINDOW_WIDTH] / 2) + 1; yaw_count_offset+=skip)
      {
         int yaw_count = projectedPoint.x + yaw_count_offset;
         if ((yaw_count >= 0) && (yaw_count < params[DEPTH_INPUT_WIDTH]) && (pitch_count >= 0) && (pitch_count < params[DEPTH_INPUT_HEIGHT]))
         {


         //TODO 50.0f should be width
         float depthValue = in[yaw_count * 50 + pitch_count].x;
         float depth = static_cast<float>(depthValue) / 1000.0f;
            float3 queryPointInSensor;
            if (params[MODE] == 0) // Spherical
            {
               queryPointInSensor = back_project_spherical(yaw_count, pitch_count, depth, params);
            }
            else if (params[MODE] == 1) // Perspective
            {
                int2 idx = make_int2(yaw_count, pitch_count);
               queryPointInSensor = back_project_perspective(idx, depth, params);
            }

            float3 queryPointInZUp = transformPoint3D32_2(
               queryPointInSensor,
               make_float3(sensorToZUpFrameTf[0], sensorToZUpFrameTf[1], sensorToZUpFrameTf[2]),
               make_float3(sensorToZUpFrameTf[4], sensorToZUpFrameTf[5], sensorToZUpFrameTf[6]),
               make_float3(sensorToZUpFrameTf[8], sensorToZUpFrameTf[9], sensorToZUpFrameTf[10]),
               make_float3(sensorToZUpFrameTf[3], sensorToZUpFrameTf[7], sensorToZUpFrameTf[11]));

            if (queryPointInZUp.x > minX && queryPointInZUp.x < maxX && queryPointInZUp.y > minY && queryPointInZUp.y < maxY)
            {
               // remove outliers before averaging for a single cell
               if (count > 1)
               {
                  currentAverageHeight = averageHeightZ / (float)(count);
                  if (fabs(queryPointInZUp.z - currentAverageHeight) > 0.1)
                  {
                     continue;
                  }
               }
               count++;
               averageHeightZ += queryPointInZUp.z;
            }
         }
      }
   }

   if (count > 0)
   {
      averageHeightZ = averageHeightZ / (float)(count);
   }
   else
   {
      // this is slightly below the floor height of what we'll accept
      averageHeightZ = -params[HEIGHT_OFFSET];
   }
   averageHeightZ = clamp(averageHeightZ, params[MIN_CLAMP_HEIGHT], params[MAX_CLAMP_HEIGHT]);
   averageHeightZ += params[HEIGHT_OFFSET];

//    write_imageui(out, (int2)(yIndex, xIndex), (uint4)((int)( (averageHeightZ) * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));

}























